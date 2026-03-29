from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, field_validator
from app.db import get_connection
from app.services.location_mapper import map_to_location, MappingError
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

    connect = get_connection()

    # Convert physical drone coordinates into logical warehouse location
    try:
        mapped_aisle, mapped_bay, mapped_level = map_to_location(scan.x, scan.y, scan.z)
    except MappingError as e:
        connect.close()
        raise HTTPException(status_code=422, detail=str(e))

    # If barcode is missing, store an exception event instead of a normal scan row.
    if not scan.palletID or scan.palletID.strip() == "":
        cursor = connect.execute(
            """
            INSERT INTO Exceptions (palletID, aisle, bay, level, reason)
            VALUES (?, ?, ?, ?, ?)
            """,
            ("", str(mapped_aisle), str(mapped_bay), mapped_level, "BARCODE_NOT_FOUND"),
        )
        exception_id = cursor.lastrowid
        connect.commit()
        connect.close()
        raise HTTPException(
            status_code=422,
            detail=f"Barcode not detected. Exception recorded with exceptionID {exception_id}."
        )

    # save mapped location and confidence to Scan table
    confidence_value = float(scan.confidence)

    cursor = connect.execute(
        "INSERT INTO Scan (palletID, aisle, bay, level, confidence) VALUES (?, ?, ?, ?, ?)",
        (scan.palletID, str(mapped_aisle), str(mapped_bay), mapped_level, confidence_value),
    )

    scan_id = cursor.lastrowid
    connect.commit()
    connect.close()

    # Return both mapped location and raw xyz for debugging purposes
    return {
        "scanID": scan_id,
        "palletID": scan.palletID,
        "aisle": mapped_aisle,
        "bay": mapped_bay,
        "level": mapped_level,
        "x": scan.x,
        "y": scan.y,
        "z": scan.z,
        "confidence": confidence_value,
    }
