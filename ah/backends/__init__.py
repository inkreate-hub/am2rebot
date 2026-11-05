"""Follower backends. Import the one you need; MuJoCo pulls in `mujoco`."""

from .real_b601 import RealB601Backend

__all__ = ["RealB601Backend"]


def __getattr__(name: str):  # lazy, so `import ah.backends` works without mujoco
    if name == "MuJoCoBackend":
        from .mujoco_backend import MuJoCoBackend

        return MuJoCoBackend
    raise AttributeError(name)
