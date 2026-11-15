"""A participant's view: a camera on their vehicle shown in a pygame window, or saved now and then headless."""
import queue

import carla
import numpy as np

from . import cockpit, look
from .compat import listen, setup_camera

VIEWS = {  # camera mounts relative to the vehicle centre
    "chase": carla.Transform(carla.Location(x=-6.5, z=2.8), carla.Rotation(pitch=-10)),
    "driver": carla.Transform(carla.Location(x=-0.05, y=-0.38, z=1.22)),   # unmeasured fallback (cockpit.py)
    "hood": carla.Transform(carla.Location(x=1.6, z=1.6)),
}


class View:
    def __init__(self, world, vehicle, view="driver", size=(1280, 720), fov=90, window=True,
                 save_dir=None, save_every_s=0.0, profile="Dashcam", capture=None, lefthand=False):
        self.window = window
        self.latest = None
        self.save_dir = save_dir
        saving = not window and save_every_s > 0
        # headless snapshots with simulation.capture (look.py): rendered at 2x and downscaled, no motion blur, an
        # exposure from the sun and clouds, rendered at 10 Hz on CARLA 0.9.x and every frame on 1.0 (its auto
        # exposure adapts), every save_every_s-th second kept here. capture None/off: on 0.9.x only the kept frames
        # are rendered, with a fixed daylight exposure (shutter 400) as before.
        cap = look.settings(capture) if (saving and capture is not None) else None
        self.out_size = tuple(size)
        rsize = look.capture_size(size, cap) if cap else tuple(size)
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(rsize[0]))
        bp.set_attribute("image_size_y", str(rsize[1]))
        bp.set_attribute("fov", str(fov))
        self.keep_every = setup_camera(bp, save_every_s if saving else 0.0, profile, 400, world=world, capture=cap)
        self._kept_at = None
        self.camera = None
        mount = VIEWS.get(view, VIEWS["driver"])
        if view == "driver" or view not in VIEWS:       # the car's measured driver's seat (cockpit.py)
            x, y, z, pitch = cockpit.actor_eye(vehicle, "driver", lefthand)
            mount = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch))
        if window or save_every_s > 0:
            # chase: CARLA's spring arm, which pulls the camera in front of a bus or a wall behind the car
            self.camera = world.spawn_actor(bp, mount, attach_to=vehicle,
                                            attachment_type=carla.AttachmentType.SpringArm if view == "chase" else
                                            carla.AttachmentType.Rigid)
            listen(self.camera, self._on_image, lambda m: print(f"[view] {m}", flush=True), "view")
        self.screen = None
        self.font = None
        if window:
            import pygame
            pygame.init()
            self.screen = pygame.display.set_mode(size, pygame.HWSURFACE | pygame.DOUBLEBUF)
            pygame.display.set_caption("coupled-carla")
            self.font = pygame.font.SysFont("monospace", 22, bold=True)

    def _on_image(self, image):
        if self.window:
            arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
            self.latest = arr[:, :, 2::-1]   # BGRA -> RGB
        elif self.save_dir:
            if self.keep_every:
                if self._kept_at is not None and image.timestamp - self._kept_at < self.keep_every - 1e-6:
                    return
                self._kept_at = image.timestamp
            look.save_frame(image, self.save_dir / f"view_{image.frame:08d}.jpg", self.out_size)

    def events(self):
        if not self.window:
            return []
        import pygame
        return pygame.event.get()

    def render(self, hud_lines=()):
        if not self.window or self.latest is None:
            return
        import pygame
        surf = pygame.surfarray.make_surface(self.latest.swapaxes(0, 1))
        self.screen.blit(surf, (0, 0))
        for i, line in enumerate(hud_lines):
            self.screen.blit(self.font.render(line, True, (255, 255, 255)), (16, 12 + 26 * i))
        pygame.display.flip()

    def destroy(self):
        if self.camera:
            try:
                self.camera.stop()
                self.camera.destroy()
            except RuntimeError:
                pass
        if self.window:
            import pygame
            pygame.quit()
