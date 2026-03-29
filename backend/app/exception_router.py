from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from sqlmodel import Session, select

from app.db import engine
from app.models import WarehouseException

router = APIRouter(prefix="/exceptions", tags=["Exceptions"])


class ExceptionCreate(BaseModel):
    palletID: str
    aisle: str
    bay: str
    level: int
    confidence: float | None = None


@router.get("/")
def get_all_exceptions():
    with Session(engine) as session:
        rows = session.exec(select(WarehouseException)).all()
    return [r.model_dump() for r in rows]


@router.get("/{exception_id}")
def get_exception(exception_id: int):
    with Session(engine) as session:
        row = session.get(WarehouseException, exception_id)
    if row is None:
        raise HTTPException(status_code=404, detail="Exception not found")
    return row.model_dump()


@router.post("/")
def create_exception(data: ExceptionCreate):
    with Session(engine) as session:
        exc = WarehouseException(
            palletID=data.palletID,
            aisle=data.aisle,
            bay=data.bay,
            level=data.level,
            reason="BARCODE_NOT_FOUND",
        )
        session.add(exc)
        session.commit()
        session.refresh(exc)
        exc_id = exc.exceptionID

    return {"message": "Exception recorded", "exceptionID": exc_id}


@router.delete("/{exception_id}")
def delete_exception(exception_id: int):
    with Session(engine) as session:
        row = session.get(WarehouseException, exception_id)
        if row is None:
            raise HTTPException(status_code=404, detail="Exception not found")
        session.delete(row)
        session.commit()

    return {"message": "Exception deleted"}
