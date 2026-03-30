import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Tuple


class MappingError(ValueError):
    """Raised when (x, y, z) cannot map to a valid warehouse location."""


@dataclass(frozen=True)
class Interval:
    start: float
    end: float

    def contains(self, value: float) -> bool:
        return self.start <= value < self.end


@dataclass(frozen=True)
class Rect:
    x_start: float
    x_end: float
    y_start: float
    y_end: float

    def contains(self, x: float, y: float) -> bool:
        return self.x_start <= x < self.x_end and self.y_start <= y < self.y_end


@dataclass(frozen=True)
class AisleBand:
    aisle_id: int
    x_range: Interval


@dataclass(frozen=True)
class LevelBand:
    level_id: int
    z_range: Interval


@dataclass(frozen=True)
class WarehouseLayout:
    # x-axis aisle bands
    aisles: List[AisleBand]
    # y-axis bay settings
    bay_origin_y: float
    bay_pitch: float
    bay_count: int
    # z-axis level bands
    levels: List[LevelBand]
    # global boundaries
    x_bounds: Interval
    y_bounds: Interval
    z_bounds: Interval
    # reserved/non-rack zones
    blocked_areas: List[Rect]


def _to_interval(raw: dict[str, Any]) -> Interval:
    return Interval(start=float(raw["start"]), end=float(raw["end"]))


def _load_layout(warehouse_id: str) -> WarehouseLayout:
    config_dir = Path(__file__).resolve().parent.parent / "configs" / "warehouses"
    config_path = config_dir / f"{warehouse_id}.json"
    if not config_path.exists():
        raise MappingError(f"unknown warehouse_id: {warehouse_id}")

    with config_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    aisles = [
        AisleBand(
            aisle_id=int(a["aisle_id"]),
            x_range=Interval(start=float(a["x_start"]), end=float(a["x_end"])),
        )
        for a in data.get("aisles", [])
    ]
    levels = [
        LevelBand(level_id=int(level["level_id"]), z_range=_to_interval(level["z_range"]))
        for level in data["levels"]
    ]
    blocked_areas = [
        Rect(
            x_start=float(area["x_start"]),
            x_end=float(area["x_end"]),
            y_start=float(area["y_start"]),
            y_end=float(area["y_end"]),
        )
        for area in data.get("blocked_areas", [])
    ]

    return WarehouseLayout(
        aisles=aisles,
        bay_origin_y=float(data["bay_origin_y"]),
        bay_pitch=float(data["bay_pitch"]),
        bay_count=int(data["bay_count"]),
        levels=levels,
        x_bounds=_to_interval(data["x_bounds"]),
        y_bounds=_to_interval(data["y_bounds"]),
        z_bounds=_to_interval(data["z_bounds"]),
        blocked_areas=blocked_areas,
    )


def get_layout(warehouse_id: str) -> WarehouseLayout:
    return _load_layout(warehouse_id)


def ensure_coordinates_map(
    x: float, y: float, z: float, warehouse_id: str = "default"
) -> None:
    """Raise ValueError if (x, y, z) cannot map to a rack cell (for API validation)."""
    try:
        map_to_location(x, y, z, warehouse_id=warehouse_id)
    except MappingError as e:
        raise ValueError(str(e)) from e


def map_to_location(
    x: float, y: float, z: float, warehouse_id: str = "default"
) -> Tuple[int, int, int]:
    # Deterministic mapping entrypoint used by backend ingestion
    layout = get_layout(warehouse_id)

    # Global bounds
    if not layout.x_bounds.contains(x):
        raise MappingError(f"x out of bounds: {x}")
    if not layout.y_bounds.contains(y):
        raise MappingError(f"y out of bounds: {y}")
    if not layout.z_bounds.contains(z):
        raise MappingError(f"z out of bounds: {z}")

    for area in layout.blocked_areas:
        if area.contains(x, y):
            raise MappingError(f"point is inside blocked area: x={x}, y={y}")

    # x -> aisle (explicit per-warehouse aisle bands from JSON)
    aisle_id = None
    for aisle in layout.aisles:
        if aisle.x_range.contains(x):
            aisle_id = aisle.aisle_id
            break
    if aisle_id is None:
        raise MappingError(f"x is outside configured aisle bands: {x}")

    # y -> bay
    relative_y = y - layout.bay_origin_y
    bay_index_zero_based = int(relative_y // layout.bay_pitch)
    bay_id = bay_index_zero_based + 1
    if bay_id < 1 or bay_id > layout.bay_count:
        raise MappingError(f"y maps outside bay range: y={y}, bay={bay_id}")

    # z -> level
    level_id = None
    for level in layout.levels:
        if level.z_range.contains(z):
            level_id = level.level_id
            break
    if level_id is None:
        raise MappingError(f"invalid rack height z={z} (no matching level)")

    return aisle_id, bay_id, level_id

