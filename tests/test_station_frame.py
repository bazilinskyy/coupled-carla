"""station.frame_surface: the shown frame comes straight from the camera's BGRA buffer, pixel for pixel the same as
the old RGB copy (pygame.surfarray.make_surface of the reversed-channel view)."""
import os

import numpy as np
import pytest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
pygame = pytest.importorskip("pygame")

from coupledcarla.station import frame_rgb_view, frame_surface  # noqa: E402


def test_bgra_buffer_shows_like_the_rgb_copy():
    rng = np.random.default_rng(3)
    w, h = 16, 9
    bgra = rng.integers(0, 256, size=(h, w, 4), dtype=np.uint8)
    raw = bgra.tobytes()
    rgb = frame_rgb_view(raw, w, h)
    assert rgb.shape == (h, w, 3) and (rgb == bgra[:, :, 2::-1]).all()
    assert not rgb.flags.owndata                                    # a view on the buffer, no copy
    new = pygame.surfarray.array3d(frame_surface(pygame, (raw, w, h), None))
    old = pygame.surfarray.array3d(pygame.surfarray.make_surface(rgb.swapaxes(0, 1)))
    assert new.shape == (w, h, 3) and (new == old).all()
    # a composited frame (no camera buffer) still goes through the RGB array
    assert (pygame.surfarray.array3d(frame_surface(pygame, None, rgb)) == old).all()
