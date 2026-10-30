"""Name -> class registries for the converter factory."""
from __future__ import annotations


class Registry:
    def __init__(self, kind: str):
        self.kind = kind
        self._items: dict[str, type] = {}

    def register(self, name: str):
        def deco(cls):
            if name in self._items:
                raise ValueError(f"{self.kind} '{name}' registered twice")
            cls.name = name
            self._items[name] = cls
            return cls
        return deco

    def create(self, name: str, **options):
        if name not in self._items:
            raise KeyError(f"unknown {self.kind} '{name}' (available: {', '.join(self.names())})")
        return self._items[name](**options)

    def names(self) -> list[str]:
        return sorted(self._items)

    def describe(self) -> list[tuple[str, str]]:
        return [(n, (self._items[n].__doc__ or "").strip().splitlines()[0]) for n in self.names()]


LIDAR = Registry("lidar decoder")
IMU = Registry("imu decoder")
GT = Registry("ground truth reader")
