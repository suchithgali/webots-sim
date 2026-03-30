from sqlalchemy import text
from sqlmodel import Session, select

from app.db import engine
from app.models import Scan, WarehouseException
from app.services.location_mapper import map_to_location, MappingError


class ScanProcessingError(Exception):
    """Service-level error that the router converts into an HTTP error."""

    def __init__(self, status_code: int, detail: str):
        super().__init__(detail)
        self.status_code = status_code
        self.detail = detail


DUPLICATE_WINDOW_SECONDS = 10


def process_scan(
    warehouse_id: str,
    pallet_id: str,
    x: float,
    y: float,
    z: float,
    confidence: float,
):
    try:
        mapped_aisle, mapped_bay, mapped_level = map_to_location(
            x, y, z, warehouse_id=warehouse_id
        )
    except MappingError as e:
        raise ScanProcessingError(status_code=422, detail=str(e))

    if not pallet_id or pallet_id.strip() == "":
        with Session(engine) as session:
            exc_row = WarehouseException(
                palletID="",
                aisle=str(mapped_aisle),
                bay=str(mapped_bay),
                level=int(mapped_level),
                reason="BARCODE_NOT_FOUND",
            )
            session.add(exc_row)
            session.commit()
            session.refresh(exc_row)
            exception_id = exc_row.exceptionID

        raise ScanProcessingError(
            status_code=422,
            detail=(
                f"Barcode not detected. Exception recorded with exceptionID {exception_id}."
            ),
        )

    with Session(engine) as session:
        existing_row = session.exec(
            select(Scan)
            .where(Scan.palletID == pallet_id)
            .where(Scan.aisle == str(mapped_aisle))
            .where(Scan.bay == str(mapped_bay))
            .where(Scan.level == mapped_level)
            .where(
                text(
                    f"datetime(Scan.timestamp) >= datetime('now', '-{DUPLICATE_WINDOW_SECONDS} seconds')"
                )
            )
            .order_by(text("scanID DESC"))
            .limit(1)
        ).first()

        if existing_row is not None:
            return {
                "scanID": existing_row.scanID,
                "palletID": pallet_id,
                "aisle": mapped_aisle,
                "bay": mapped_bay,
                "level": mapped_level,
                "x": x,
                "y": y,
                "z": z,
                "confidence": existing_row.confidence,
                "duplicate": True,
            }

        confidence_value = float(confidence)
        row = Scan(
            palletID=pallet_id,
            aisle=str(mapped_aisle),
            bay=str(mapped_bay),
            level=int(mapped_level),
            confidence=confidence_value,
        )
        session.add(row)
        session.commit()
        session.refresh(row)
        scan_id = row.scanID

    return {
        "scanID": scan_id,
        "palletID": pallet_id,
        "aisle": mapped_aisle,
        "bay": mapped_bay,
        "level": mapped_level,
        "x": x,
        "y": y,
        "z": z,
        "confidence": confidence_value,
        "duplicate": False,
    }
