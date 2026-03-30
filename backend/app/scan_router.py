from typing import Any, Self

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field, field_validator, model_validator
from sqlmodel import Session, select

from app.db import engine
from app.models import Scan
from app.services.location_mapper import ensure_coordinates_map, get_layout
from app.services.scan_service import ScanProcessingError, process_scan

router = APIRouter(prefix="/scans")
DEFAULT_LAYOUT = get_layout("default")


class ScanCreate(BaseModel):
    """Scan payload; x/y/z are in warehouse inches (same units as ``location_mapper``)."""

    warehouseID: str = Field(
        default="default",
        min_length=1,
        description="Warehouse profile ID used to load mapping rules.",
    )
    palletID: str = Field(..., description="Barcode / license plate; may be empty for no-read.")
    x: float = Field(
        ...,
        ge=DEFAULT_LAYOUT.x_bounds.start,
        lt=DEFAULT_LAYOUT.x_bounds.end,
        description="X position (inches); must fall inside a rack aisle band.",
    )
    y: float = Field(
        ...,
        ge=DEFAULT_LAYOUT.y_bounds.start,
        lt=DEFAULT_LAYOUT.y_bounds.end,
        description="Y position (inches) along bay depth.",
    )
    z: float = Field(
        ...,
        ge=DEFAULT_LAYOUT.z_bounds.start,
        lt=DEFAULT_LAYOUT.z_bounds.end,
        description="Z height (inches); must match a defined rack level band.",
    )
    confidence: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Read confidence; >= 0.99 required when palletID is non-empty.",
    )

    @field_validator("palletID")
    @classmethod
    def validate_pallet_id(cls, v: str) -> str:
        return v.strip()

    @field_validator("warehouseID")
    @classmethod
    def validate_warehouse_id(cls, v: str) -> str:
        return v.strip()

    @field_validator("confidence")
    @classmethod
    def validate_confidence(cls, v: float, info: Any) -> float:
        if v < 0.0 or v > 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")

        pallet_id = info.data.get("palletID", "")
        if pallet_id != "" and v < 0.99:
            raise ValueError("confidence must be >= 0.99")

        return float(v)

    @model_validator(mode="after")
    def coordinates_must_map_to_rack(self) -> Self:
        ensure_coordinates_map(self.x, self.y, self.z, warehouse_id=self.warehouseID)
        return self


@router.get("/", status_code=200)
def list_scans():
    with Session(engine) as session:
        scans = session.exec(select(Scan)).all()
    return [s.model_dump() for s in scans]


@router.get("/{scan_id}", status_code=200)
def get_scan(scan_id: int):
    with Session(engine) as session:
        row = session.get(Scan, scan_id)
    if row is None:
        raise HTTPException(status_code=404, detail="Scan not found")
    return row.model_dump()


@router.post("/", status_code=201)
def create_scan(scan: ScanCreate):
    try:
        return process_scan(
            warehouse_id=scan.warehouseID,
            pallet_id=scan.palletID,
            x=scan.x,
            y=scan.y,
            z=scan.z,
            confidence=scan.confidence,
        )
    except ScanProcessingError as e:
        raise HTTPException(status_code=e.status_code, detail=e.detail)
