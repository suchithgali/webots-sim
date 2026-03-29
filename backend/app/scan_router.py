from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, field_validator
from sqlmodel import Session, select

from app.db import engine
from app.models import Scan
from app.services.scan_service import ScanProcessingError, process_scan

router = APIRouter(prefix="/scans")


class ScanCreate(BaseModel):
    palletID: str
    x: float
    y: float
    z: float
    confidence: float

    @field_validator("palletID")
    def validate_pallet_id(cls, v: str) -> str:
        return v.strip()

    @field_validator("confidence")
    def validate_confidence(cls, v: float, info: Any) -> float:
        if v < 0.0 or v > 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")

        pallet_id = info.data.get("palletID", "")
        if pallet_id != "" and v < 0.99:
            raise ValueError("confidence must be >= 0.99")

        return float(v)


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
            pallet_id=scan.palletID,
            x=scan.x,
            y=scan.y,
            z=scan.z,
            confidence=scan.confidence,
        )
    except ScanProcessingError as e:
        raise HTTPException(status_code=e.status_code, detail=e.detail)
