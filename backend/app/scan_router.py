from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, field_validator
from app.db import get_connection
from app.services.scan_service import process_scan, ScanProcessingError
from typing import Any

# Router for all scan-related API endpoints
router = APIRouter(prefix="/scans")

# Request body structure for creating a scan
# Drone sends xyz + palletID/confidence; backend maps xyz -> aisle/bay/level.
class ScanCreate(BaseModel):
    palletID: str
    x: float
    y: float
    z: float
    confidence: float

    @field_validator("palletID")
    def validate_pallet_id(cls, v: str) -> str:
        # Allow empty palletID so the endpoint can record BARCODE_NOT_FOUND exceptions
        return v.strip()

    @field_validator("confidence")
    def validate_confidence(cls, v: float, info: Any) -> float:
        if v < 0.0 or v > 1.0:
            raise ValueError("confidence must be between 0.0 and 1.0")

        # If we have a pallet barcode, enforce 99%+ confidence as required by the project
        pallet_id = info.data.get("palletID", "")
        if pallet_id != "" and v < 0.99:
            raise ValueError("confidence must be >= 0.99")

        return float(v)

# Read all rows from the Scan table and return them as a list of JSON objects
@router.get("/", status_code=200)
def list_scans():
    connect = get_connection()
    rows = connect.execute("SELECT * FROM Scan").fetchall()
    connect.close()
    return [dict(row) for row in rows]

# Read a single row from the Scan table by scanID
@router.get("/{scan_id}", status_code=200)
def get_scan(scan_id: int):
    connect = get_connection()
    try:
        row = connect.execute(
            "SELECT * FROM Scan WHERE scanID = ?", (scan_id,)
        ).fetchone()
    finally:
        connect.close()

    if row is None:
        raise HTTPException(status_code=404, detail="Scan not found")

    return dict(row)

# Insert a new row into the scan table and return that new scan as JSON
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
