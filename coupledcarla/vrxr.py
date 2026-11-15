"""The OpenXR side of a VR station (vr.py): a headset session in its own thread, through pyopenxr.

    pip install -e ".[vr]"        # pyopenxr (Apache-2.0), glfw (zlib), PyOpenGL (BSD)

The thread owns an OpenGL context (a hidden GLFW window) and the OpenXR session (OpenGL binding: GLX on Linux,
WGL on Windows; every PC runtime supports it: Meta Quest Link / Air Link, SteamVR, Varjo, WMR, Pico, Monado). At the
headset's rate it waits for the frame, locates the head (VIEW space in LOCAL or STAGE space, with velocities) at the
predicted display time for the station, uploads the newest pair of eye images the station submitted and ends the
frame with one projection layer whose views carry the poses the images were rendered at, so the runtime reprojects
them to the head pose at display time. Between CARLA frames the last pair is submitted again (reprojected).

python -m coupledcarla.vrxr prints the runtime, the headset, its per-eye resolution and field of view, and runs a
few seconds of frames with test images (no CARLA): the check that a lab PC's VR setup works.
"""
import ctypes
import math
import os
import sys
import threading
import time

from . import vr

INSTALL = ("VR needs pyopenxr, glfw and PyOpenGL (pip install -e \".[vr]\") and an OpenXR runtime: Meta Quest Link, "
           "SteamVR, Varjo Base or Monado (docs/lab_guide.md, VR)")


def _import():
    try:
        import glfw  # noqa: F401
        import xr  # noqa: F401
        from OpenGL import GL  # noqa: F401
    except ImportError as e:
        raise RuntimeError(f"{INSTALL} ({e})") from e


def _glx_binding(xr):
    """The XLib binding of the current GLX context, with its framebuffer config and visual (Monado needs both; pyopenxr's
    own helper leaves them empty)."""
    from OpenGL import GLX
    dpy, ctx, drw = GLX.glXGetCurrentDisplay(), GLX.glXGetCurrentContext(), GLX.glXGetCurrentDrawable()
    fid = ctypes.c_int()
    GLX.glXQueryContext(dpy, ctx, GLX.GLX_FBCONFIG_ID, ctypes.byref(fid))
    screen = ctypes.c_int()
    GLX.glXQueryContext(dpy, ctx, GLX.GLX_SCREEN, ctypes.byref(screen))
    attrs = (ctypes.c_int * 3)(GLX.GLX_FBCONFIG_ID, fid.value, 0)
    n = ctypes.c_int()
    cfgs = GLX.glXChooseFBConfig(dpy, screen.value, attrs, ctypes.byref(n))
    if not n.value:
        raise RuntimeError("no GLX framebuffer config for the OpenGL context")
    vid = ctypes.c_int()
    GLX.glXGetFBConfigAttrib(dpy, cfgs[0], GLX.GLX_VISUAL_ID, ctypes.byref(vid))
    return xr.GraphicsBindingOpenGLXlibKHR(x_display=dpy, visualid=vid.value, glx_fbconfig=cfgs[0],
                                           glx_drawable=drw, glx_context=ctx)


class XrHeadset:
    """vr.SimHeadset's interface on a real OpenXR runtime: start(), fov, recommended, ipd_m, pose(), submit(),
    first_shown(), stop(); info has the runtime's and the headset's names."""

    def __init__(self, cfg, log=print):
        _import()
        self.cfg, self.log = cfg, log
        self.fov = None
        self.recommended = None
        self.ipd_m = None
        self.info = {}
        self.ready = threading.Event()
        self.stopped = threading.Event()
        self.error = None
        self.lock = threading.Lock()
        self._pose = None
        self._new = None             # (image id, [left, right] BGRA, xr poses, fovs) waiting for upload
        self._shown_img = None       # (image id, xr poses, fovs, (w, h)) on the swapchains' source textures
        self.shown = {}              # image id -> (display time, head pose then)
        self.frames = 0
        self.thread = None

    # -- the station's side -----------------------------------------------------------------------------------
    def start(self, timeout=30.0):
        self.thread = threading.Thread(target=self._run, name="openxr", daemon=True)
        self.thread.start()
        self.ready.wait(timeout)
        if self.error is not None:
            raise RuntimeError(f"OpenXR: {self.error}")
        if not self.ready.is_set():
            raise RuntimeError(f"OpenXR: the session did not start in {timeout:.0f} s (is the headset on and the "
                               "runtime set as the active OpenXR runtime?)")
        return True

    def pose(self):
        with self.lock:
            return self._pose if self._pose is not None else (time.perf_counter(), time.perf_counter(), None, None,
                                                              None)

    def submit(self, image_id, eyes, poses_xr, fovs):
        with self.lock:
            self._new = (image_id, eyes, poses_xr, fovs)

    def first_shown(self, image_id):
        with self.lock:
            return self.shown.pop(image_id, None)

    def stop(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(5.0)

    # -- the thread ---------------------------------------------------------------------------------------------
    def _run(self):
        try:
            self._setup()
            self._loop()
        except Exception as e:  # noqa: BLE001 - reported to the station
            self.error = f"{type(e).__name__}: {e}"
            self.log(f"vr: OpenXR stopped: {self.error}")
            self.ready.set()
        finally:
            try:
                self._teardown()
            except Exception:  # noqa: BLE001
                pass

    def _setup(self):
        import glfw
        import xr
        from OpenGL import GL
        self.xr, self.GL, self.glfw = xr, GL, glfw
        names = []
        for e in xr.enumerate_instance_extension_properties():
            n = e.extension_name
            names.append(n.decode() if isinstance(n, bytes) else str(n))
        want = [xr.KHR_OPENGL_ENABLE_EXTENSION_NAME]
        self.time_ext = None
        for ext in ("XR_KHR_convert_timespec_time", "XR_KHR_win32_convert_performance_counter_time"):
            if ext in names:
                want.append(ext)
                self.time_ext = ext
        if xr.KHR_OPENGL_ENABLE_EXTENSION_NAME not in names:
            raise RuntimeError(f"the runtime has no OpenGL support ({xr.KHR_OPENGL_ENABLE_EXTENSION_NAME})")
        self.instance = xr.create_instance(xr.InstanceCreateInfo(
            application_info=xr.ApplicationInfo(application_name="coupled-carla station", application_version=1,
                                                engine_name="coupled-carla", engine_version=1,
                                                api_version=xr.Version(1, 0, xr.XR_VERSION_PATCH)),
            enabled_extension_names=want))
        props = xr.get_instance_properties(self.instance)
        rn = props.runtime_name
        self.info["runtime"] = (rn.decode() if isinstance(rn, bytes) else str(rn)) + \
            f" {xr.Version(props.runtime_version)}" if hasattr(xr, "Version") else ""
        self.system = xr.get_system(self.instance, xr.SystemGetInfo(form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY))
        sp = xr.get_system_properties(self.instance, self.system)
        sn = sp.system_name
        self.info["headset"] = sn.decode() if isinstance(sn, bytes) else str(sn)
        self.view_type = xr.ViewConfigurationType.PRIMARY_STEREO
        views = xr.enumerate_view_configuration_views(self.instance, self.system, self.view_type)
        self.recommended = (int(views[0].recommended_image_rect_width), int(views[0].recommended_image_rect_height))
        self.info["recommended"] = f"{self.recommended[0]}x{self.recommended[1]}"
        # an OpenGL context in this thread: a hidden GLFW window (GLX on Linux, WGL on Windows)
        if not glfw.init():
            raise RuntimeError("glfw.init failed (no display? on Linux run under X or Xvfb)")
        glfw.window_hint(glfw.VISIBLE, False)
        glfw.window_hint(glfw.DOUBLEBUFFER, False)
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 4)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 1)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        self.window = glfw.create_window(64, 64, "coupled-carla vr", None, None)
        if not self.window:
            raise RuntimeError("could not create an OpenGL 4.1 context")
        glfw.make_context_current(self.window)
        glfw.swap_interval(0)
        req = xr.GraphicsRequirementsOpenGLKHR()
        fn = ctypes.cast(xr.get_instance_proc_addr(self.instance, "xrGetOpenGLGraphicsRequirementsKHR"),
                         xr.PFN_xrGetOpenGLGraphicsRequirementsKHR)
        res = xr.check_result(xr.Result(fn(self.instance, self.system, ctypes.byref(req))))
        if res.is_exception():
            raise res
        if os.name == "nt":
            from OpenGL import WGL
            self.binding = xr.GraphicsBindingOpenGLWin32KHR()
            self.binding.h_dc = WGL.wglGetCurrentDC()
            self.binding.h_glrc = WGL.wglGetCurrentContext()
        else:
            self.binding = _glx_binding(xr)
        self.session = xr.create_session(self.instance, xr.SessionCreateInfo(
            system_id=self.system, next=ctypes.cast(ctypes.pointer(self.binding), ctypes.c_void_p)))
        ref = {"stage": xr.ReferenceSpaceType.STAGE}.get(str(self.cfg.get("reference", "local")).lower(),
                                                          xr.ReferenceSpaceType.LOCAL)
        self.space = xr.create_reference_space(self.session, xr.ReferenceSpaceCreateInfo(
            reference_space_type=ref, pose_in_reference_space=xr.Posef()))
        self.view_space = xr.create_reference_space(self.session, xr.ReferenceSpaceCreateInfo(
            reference_space_type=xr.ReferenceSpaceType.VIEW, pose_in_reference_space=xr.Posef()))
        fmts = list(xr.enumerate_swapchain_formats(self.session))
        self.format = next((f for f in (GL.GL_SRGB8_ALPHA8, GL.GL_RGBA8) if f in fmts), fmts[0] if fmts else None)
        self.info["format"] = {GL.GL_SRGB8_ALPHA8: "SRGB8_ALPHA8", GL.GL_RGBA8: "RGBA8"}.get(self.format,
                                                                                         hex(self.format or 0))
        self.swapchains = None
        self.src_tex = GL.glGenTextures(2)
        self.fbo_read, self.fbo_draw = GL.glGenFramebuffers(2)
        self.src_size = None
        self.running = False
        self.state = None
        self._to_perf = self._time_converter()
        self.log(f"vr: OpenXR runtime {self.info.get('runtime')}, headset {self.info['headset']}, "
                 f"recommended {self.info['recommended']} per eye, swapchain {self.info['format']}")

    def _time_converter(self):
        """XrTime -> time.perf_counter() seconds (both CLOCK_MONOTONIC on Linux, the performance counter on
        Windows), or None without the runtime's conversion extension."""
        xr = self.xr
        try:
            if self.time_ext == "XR_KHR_convert_timespec_time":
                from xr.platform.linux import PFN_xrConvertTimeToTimespecTimeKHR, timespec
                fn = ctypes.cast(xr.get_instance_proc_addr(self.instance, "xrConvertTimeToTimespecTimeKHR"),
                                 PFN_xrConvertTimeToTimespecTimeKHR)

                def conv(t):
                    ts = timespec()
                    fn(self.instance, t, ctypes.byref(ts))
                    return ts.tv_sec + ts.tv_nsec * 1e-9
                return conv
            if self.time_ext == "XR_KHR_win32_convert_performance_counter_time":
                from xr.platform.windows import PFN_xrConvertTimeToWin32PerformanceCounterKHR
                fn = ctypes.cast(xr.get_instance_proc_addr(self.instance,
                                                           "xrConvertTimeToWin32PerformanceCounterKHR"),
                                 PFN_xrConvertTimeToWin32PerformanceCounterKHR)
                freq = ctypes.c_int64()
                ctypes.windll.kernel32.QueryPerformanceFrequency(ctypes.byref(freq))

                def conv(t):
                    c = ctypes.c_int64()
                    fn(self.instance, t, ctypes.byref(c))
                    return c.value / freq.value
                return conv
        except Exception as e:  # noqa: BLE001
            self.log(f"vr: no XrTime conversion ({e}); display times estimated")
        return None

    def _events(self):
        xr = self.xr
        while True:
            try:
                ev = xr.poll_event(self.instance)
            except xr.EventUnavailable:
                return
            kind = xr.StructureType(ev.type)
            if kind == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                e = ctypes.cast(ctypes.byref(ev), ctypes.POINTER(xr.EventDataSessionStateChanged)).contents
                self.state = xr.SessionState(e.state)
                if self.state == xr.SessionState.READY:
                    xr.begin_session(self.session, xr.SessionBeginInfo(self.view_type))
                    self.running = True
                elif self.state == xr.SessionState.STOPPING:
                    xr.end_session(self.session)
                    self.running = False
                elif self.state in (xr.SessionState.EXITING, xr.SessionState.LOSS_PENDING):
                    self.stopped.set()
            elif kind == xr.StructureType.EVENT_DATA_INSTANCE_LOSS_PENDING:
                self.stopped.set()

    def _swapchains(self, w, h):
        xr, GL = self.xr, self.GL
        for sc in (self.swapchains or []):
            xr.destroy_swapchain(sc[0])
        self.swapchains = []
        for _ in range(2):
            sc = xr.create_swapchain(self.session, xr.SwapchainCreateInfo(
                usage_flags=xr.SwapchainUsageFlags.COLOR_ATTACHMENT_BIT | xr.SwapchainUsageFlags.TRANSFER_DST_BIT,
                format=self.format, sample_count=1, width=w, height=h, face_count=1, array_size=1, mip_count=1))
            imgs = xr.enumerate_swapchain_images(sc, xr.SwapchainImageOpenGLKHR)
            self.swapchains.append((sc, [i.image for i in imgs], imgs))
        for tex in self.src_tex:
            GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
            GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGBA8, w, h, 0, GL.GL_BGRA, GL.GL_UNSIGNED_BYTE, None)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        self.src_size = (w, h)
        self.log(f"vr: swapchains {w}x{h} per eye")

    def _upload(self):
        GL = self.GL
        with self.lock:
            new, self._new = self._new, None
        if new is None:
            return
        image_id, eyes, poses, fovs = new
        h, w = eyes[0].shape[:2]
        if self.src_size != (w, h):
            self._swapchains(w, h)
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 4)
        for tex, img in zip(self.src_tex, eyes):
            GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
            GL.glTexSubImage2D(GL.GL_TEXTURE_2D, 0, 0, 0, w, h, GL.GL_BGRA, GL.GL_UNSIGNED_BYTE, img)
        self._shown_img = (image_id, poses, fovs, (w, h))

    def _layer(self):
        """Copies the source textures into the swapchains (flipped: OpenGL's rows start at the bottom) and returns
        the projection layer, or None before the first image."""
        xr, GL = self.xr, self.GL
        if self._shown_img is None:
            return None
        image_id, poses, fovs, (w, h) = self._shown_img
        views = []
        for i, (sc, texs, _keep) in enumerate(self.swapchains):
            idx = xr.acquire_swapchain_image(sc, xr.SwapchainImageAcquireInfo())
            xr.wait_swapchain_image(sc, xr.SwapchainImageWaitInfo(timeout=xr.INFINITE_DURATION))
            GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER, self.fbo_read)
            GL.glFramebufferTexture2D(GL.GL_READ_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0, GL.GL_TEXTURE_2D,
                                      self.src_tex[i], 0)
            GL.glBindFramebuffer(GL.GL_DRAW_FRAMEBUFFER, self.fbo_draw)
            GL.glFramebufferTexture2D(GL.GL_DRAW_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0, GL.GL_TEXTURE_2D, texs[idx], 0)
            GL.glBlitFramebuffer(0, 0, w, h, 0, h, w, 0, GL.GL_COLOR_BUFFER_BIT, GL.GL_NEAREST)
            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0)
            xr.release_swapchain_image(sc, xr.SwapchainImageReleaseInfo())
            p, q = poses[i]
            fl, fr, fu, fd = fovs[i]
            v = xr.CompositionLayerProjectionView(
                pose=xr.Posef(orientation=xr.Quaternionf(*q), position=xr.Vector3f(*p)),
                fov=xr.Fovf(angle_left=fl, angle_right=fr, angle_up=fu, angle_down=fd))
            v.sub_image.swapchain = sc
            v.sub_image.image_rect.offset[:] = [0, 0]
            v.sub_image.image_rect.extent[:] = [w, h]
            views.append(v)
        GL.glFlush()
        return xr.CompositionLayerProjection(space=self.space, views=views), image_id

    def _locate(self, t_xr):
        xr = self.xr
        vel = xr.SpaceVelocity()
        loc = xr.SpaceLocation(next=ctypes.cast(ctypes.pointer(vel), ctypes.c_void_p))
        res = xr.check_result(xr.Result(xr.raw_functions.xrLocateSpace(self.view_space, self.space, t_xr,
                                                                      ctypes.byref(loc))))
        if res.is_exception():
            raise res
        flags = int(loc.location_flags)
        if not flags & int(xr.SPACE_LOCATION_ORIENTATION_VALID_BIT):
            return None
        o, p = loc.pose.orientation, loc.pose.position
        head = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        vf = int(vel.velocity_flags)
        lv = (vel.linear_velocity.x, vel.linear_velocity.y, vel.linear_velocity.z) \
            if vf & int(xr.SPACE_VELOCITY_LINEAR_VALID_BIT) else None
        av = (vel.angular_velocity.x, vel.angular_velocity.y, vel.angular_velocity.z) \
            if vf & int(xr.SPACE_VELOCITY_ANGULAR_VALID_BIT) else None
        return head, lv, av

    def _loop(self):
        xr = self.xr
        t_start = time.perf_counter()
        while not self.stopped.is_set():
            self._events()
            if not self.running:
                if time.perf_counter() - t_start > 30 and not self.ready.is_set():
                    raise RuntimeError(f"session never became ready (state {self.state})")
                time.sleep(0.02)
                continue
            fs = xr.wait_frame(self.session)
            t_sample = time.perf_counter()
            xr.begin_frame(self.session)
            period = fs.predicted_display_period / 1e9 if fs.predicted_display_period else 1 / 90
            disp = self._to_perf(fs.predicted_display_time) if self._to_perf else t_sample + period
            got = self._locate(fs.predicted_display_time)
            if self.fov is None or self.ipd_m is None:
                vs, views = xr.locate_views(self.session, xr.ViewLocateInfo(
                    view_configuration_type=self.view_type, display_time=fs.predicted_display_time,
                    space=self.space))
                if len(views) >= 2:
                    fl = min(v.fov.angle_left for v in views)
                    fr = max(v.fov.angle_right for v in views)
                    fu = max(v.fov.angle_up for v in views)
                    fd = min(v.fov.angle_down for v in views)
                    self.fov = (fl, fr, fu, fd)
                    a, b = views[0].pose.position, views[1].pose.position
                    self.ipd_m = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) or 0.063
                    self.info["fov_deg"] = [round(math.degrees(fr - fl), 1), round(math.degrees(fu - fd), 1)]
                    self.info["runtime_ipd_mm"] = round(1000 * self.ipd_m, 1)
            if got is not None:
                head, lv, av = got
                with self.lock:
                    self._pose = (t_sample, disp, head, lv, av)
            layers = []
            if fs.should_render:
                self._upload()
                out = self._layer()
                if out is not None:
                    layer, image_id = out
                    layers.append(ctypes.byref(layer))
                    with self.lock:
                        if image_id not in self.shown and len(self.shown) < 500:
                            self.shown[image_id] = (disp, got[0] if got is not None else None)
            xr.end_frame(self.session, xr.FrameEndInfo(display_time=fs.predicted_display_time,
                                                       environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE,
                                                       layers=layers))
            self.frames += 1
            if not self.ready.is_set() and self.fov is not None and got is not None:
                self.ready.set()

    def _teardown(self):
        xr = getattr(self, "xr", None)
        if xr is None:
            return
        for sc in (self.swapchains or []):
            xr.destroy_swapchain(sc[0])
        for name in ("view_space", "space"):
            if getattr(self, name, None) is not None:
                xr.destroy_space(getattr(self, name))
        if getattr(self, "session", None) is not None:
            if self.running:
                try:
                    xr.request_exit_session(self.session)
                except Exception:  # noqa: BLE001
                    pass
            xr.destroy_session(self.session)
        if getattr(self, "instance", None) is not None:
            xr.destroy_instance(self.instance)
        if getattr(self, "window", None) is not None:
            self.glfw.destroy_window(self.window)
            self.glfw.terminate()


def selftest(seconds=5.0, size=None, log=print):
    """Runs a headset session with test images (a grid with the frame number's colour) for `seconds`; returns
    {info, frames, submitted, fps, ...}. No CARLA."""
    import numpy as np
    cfg = vr.settings({"runtime": "openxr"})
    hs = XrHeadset(cfg, log)
    hs.start()
    cam = vr.eye_camera(hs.fov, hs.recommended[0], 1.0 if size is None else 1.0, 0.1, None,
                        size and [size[0], size[1]])
    w, h = cam["width"], cam["height"]
    base = np.zeros((h, w, 4), np.uint8)
    base[::32, :, :3] = 200
    base[:, ::32, :3] = 200
    base[..., 3] = 255
    t0 = time.perf_counter()
    n = 0
    m2p = []
    while time.perf_counter() - t0 < seconds:
        _ts, disp, head, _v, _a = hs.pose()
        if head is None:
            time.sleep(0.005)
            continue
        img = base.copy()
        img[h // 2 - 20:h // 2 + 20, :, 1] = (n * 7) % 255
        hs.submit(n, [img, img], vr.eye_poses_xr(head, 0.063), (cam["layer_fov"], cam["layer_fov"]))
        n += 1
        time.sleep(1 / 45)
        for k in list(range(max(0, n - 10), n)):
            s = hs.first_shown(k)
            if s is not None:
                m2p.append(1000 * (s[0] - _ts))
    frames = hs.frames
    hs.stop()
    return {"info": hs.info, "frames": frames, "seconds": seconds, "runtime_fps": round(frames / seconds, 1),
            "submitted": n, "size": f"{w}x{h}", "fov": [round(math.degrees(a), 1) for a in hs.fov],
            "display_minus_sample_ms": round(sorted(m2p)[len(m2p) // 2], 1) if m2p else None}


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="Check the OpenXR runtime and headset of this PC (no CARLA).")
    ap.add_argument("--seconds", type=float, default=5.0)
    a = ap.parse_args(argv)
    try:
        out = selftest(a.seconds)
    except RuntimeError as e:
        print(e, file=sys.stderr)
        return 2
    for k, v in out.items():
        print(f"{k}: {v}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
