from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional
from app.db import get_connection

# Router for all scan-related API endpoints
router = APIRouter(prefix="/scans")

# Request body structure for creating a scan
class ScanCreate(BaseModel):
    palletID: str = Field(..., example="PAL123")
    aisle: str = Field(..., example="A1")
    bay: str = Field(..., example="B2")
    level: int = Field(..., ge=0, example=1)
    confidence: Optional[float] = Field(default=1.0, ge=0, le=1)


# Insert a new row into the scan table and return that new scan as JSON
@router.post("/", status_code=201)
def create_scan(scan: ScanCreate):

    connect = get_connection()

    # trigger exception if barcode not detected
    if not scan.palletID or scan.palletID.strip() == "":
        connect.execute(
            """
            INSERT INTO Exceptions (aisle, bay, level, reason)
            VALUES (?, ?, ?, ?)
            """,
            (scan.aisle, scan.bay, scan.level, "BARCODE_NOT_FOUND"),
        )

        connect.commit()
        connect.close()

        raise HTTPException(
            status_code=400,
            detail="Barcode not detected. Exception recorded."
        )

    # normal scan logic
    if scan.confidence is None:
        confidence_value = 1.0
    else:
        confidence_value = float(scan.confidence)

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


# Read all rows from the Scan table and return them as a list of JSON objects
@router.get("/", status_code=200)
def list_scans():
    connect = get_connection()
    rows = connect.execute("SELECT * FROM Scan").fetchall()
    connect.close()

    result = []
    for row in rows:
        result.append(dict(row))

    return result