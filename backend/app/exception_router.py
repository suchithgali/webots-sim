from fastapi import APIRouter
from pydantic import BaseModel
from datetime import datetime

# --- CHANGE ---
# fixed database import to match rest of project
from app.db import get_connection

router = APIRouter(prefix="/exceptions", tags=["Exceptions"])


class ExceptionCreate(BaseModel):
    palletID: str
    aisle: str
    bay: str
    level: int
    confidence: float | None = None  


# GET all exceptions
@router.get("/")
def get_all_exceptions():

    db = get_connection()

    rows = db.execute(
        "SELECT * FROM Exceptions"
    ).fetchall()

    db.close()

    return [dict(row) for row in rows]


# GET a specific exception
@router.get("/{exception_id}")
def get_exception(exception_id: int):

    db = get_connection()

    row = db.execute(
        "SELECT * FROM Exceptions WHERE exceptionID = ?",
        (exception_id,)
    ).fetchone()

    db.close()

    if row is None:
        return {"error": "Exception not found"}

    return dict(row)


# CREATE a new exception
@router.post("/")
def create_exception(data: ExceptionCreate):

    db = get_connection()

    db.execute(
        """
        INSERT INTO Exceptions (palletID, aisle, bay, level, reason)
        VALUES (?, ?, ?, ?, ?)
        """,
        (
            data.palletID,
            data.aisle,
            data.bay,
            data.level,
            "BARCODE_NOT_FOUND"
        )
    )

    db.commit()
    db.close()

    return {"message": "Exception recorded"}


# DELETE exception (useful for clearing logs)
@router.delete("/{exception_id}")
def delete_exception(exception_id: int):

    db = get_connection()

    db.execute(
        "DELETE FROM Exceptions WHERE exceptionID = ?",
        (exception_id,)
    )

    db.commit()
    db.close()

    return {"message": "Exception deleted"}