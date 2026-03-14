from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional
from app.db import get_connection

# Router for all scan-related API endpoints
router = APIRouter(prefix="/scans")

# Request body structure for creating a scan
class ScanCreate(BaseModel):
    palletID: str = Field(..., min_length=1, example="PAL123")
    aisle: str = Field(..., example="A1")
    bay: str = Field(..., example="B2")
    level: int = Field(..., ge=0, example=1)
    confidence: Optional[float] = Field(default=1.0, ge=0, le=1)

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

    # trigger exception if barcode not detected
    if not scan.palletID or scan.palletID.strip() == "":
        cursor = connect.execute(
            """
            INSERT INTO Exceptions (aisle, bay, level, reason)
            VALUES (?, ?, ?, ?)
            """,
            (scan.aisle, scan.bay, scan.level, "BARCODE_NOT_FOUND"),
        )
        exception_id = cursor.lastrowid
        connect.commit()
        connect.close()
        raise HTTPException(
            status_code=422,
            detail=f"Barcode not detected. Exception recorded with exceptionID {exception_id}."
        )

    # normal scan logic
    confidence_value = float(scan.confidence) if scan.confidence is not None else 1.0

    cursor = connect.execute(
        "INSERT INTO Scan (palletID, aisle, bay, level, confidence) VALUES (?, ?, ?, ?, ?)",
        (scan.palletID, scan.aisle, scan.bay, scan.level, confidence_value),
    )

    scan_id = cursor.lastrowid
    connect.commit()
    connect.close()

    return {
        "scanID": scan_id,
        "palletID": scan.palletID,
        "aisle": scan.aisle,
        "bay": scan.bay,
        "level": scan.level,
        "confidence": confidence_value,
    }
