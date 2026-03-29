from dataclasses import dataclass
from typing import List, Tuple


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


def build_reference_layout() -> WarehouseLayout:
    aisles: List[AisleBand] = []

    # Reference values from layout notes (inches); tune these per warehouse
    aisle_width = 100.0
    aisle_gap = 128.0
    aisle_count = 22
    x = 0.0

    for aisle_id in range(1, aisle_count + 1):
        aisles.append(AisleBand(aisle_id=aisle_id, x_range=Interval(x, x + aisle_width)))
        x = x + aisle_width + aisle_gap

    # Vertical shelf bands (z -> level)
    levels = [
        LevelBand(level_id=1, z_range=Interval(0.0, 72.0)),
        LevelBand(level_id=2, z_range=Interval(72.0, 144.0)),
        LevelBand(level_id=3, z_range=Interval(144.0, 216.0)),
        LevelBand(level_id=4, z_range=Interval(216.0, 288.0)),
    ]

    # Example blocked floor zones (ex penthouse areas)
    blocked = [
        Rect(x_start=1200.0, x_end=1540.0, y_start=1800.0, y_end=2140.0),
        Rect(x_start=3400.0, x_end=3740.0, y_start=1800.0, y_end=2140.0),
    ]

    return WarehouseLayout(
        aisles=aisles,
        bay_origin_y=0.0,
        bay_pitch=100.0,
        bay_count=60,
        levels=levels,
        x_bounds=Interval(0.0, x),
        y_bounds=Interval(0.0, 6000.0),
        z_bounds=Interval(0.0, 288.0),
        blocked_areas=blocked,
    )


LAYOUT = build_reference_layout()


def ensure_coordinates_map(x: float, y: float, z: float) -> None:
    """Raise ValueError if (x, y, z) cannot map to a rack cell (for API validation)."""
    try:
        map_to_location(x, y, z)
    except MappingError as e:
        raise ValueError(str(e)) from e


def map_to_location(x: float, y: float, z: float) -> Tuple[int, int, int]:
    # Deterministic mapping entrypoint used by backend ingestion
    # Global bounds
    if not LAYOUT.x_bounds.contains(x):
        raise MappingError(f"x out of bounds: {x}")
    if not LAYOUT.y_bounds.contains(y):
        raise MappingError(f"y out of bounds: {y}")
    if not LAYOUT.z_bounds.contains(z):
        raise MappingError(f"z out of bounds: {z}")

    for area in LAYOUT.blocked_areas:
        if area.contains(x, y):
            raise MappingError(f"point is inside blocked area: x={x}, y={y}")

    # x -> aisle
    aisle_id = None
    for aisle in LAYOUT.aisles:
        if aisle.x_range.contains(x):
            aisle_id = aisle.aisle_id
            break
    if aisle_id is None:
        raise MappingError(f"x is between aisles: {x}")

    # y -> bay
    relative_y = y - LAYOUT.bay_origin_y
    bay_index_zero_based = int(relative_y // LAYOUT.bay_pitch)
    bay_id = bay_index_zero_based + 1
    if bay_id < 1 or bay_id > LAYOUT.bay_count:
        raise MappingError(f"y maps outside bay range: y={y}, bay={bay_id}")

    # z -> level
    level_id = None
    for level in LAYOUT.levels:
        if level.z_range.contains(z):
            level_id = level.level_id
            break
    if level_id is None:
        raise MappingError(f"invalid rack height z={z} (no matching level)")

    return aisle_id, bay_id, level_id

