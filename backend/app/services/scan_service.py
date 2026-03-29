from app.db import get_connection
from app.services.location_mapper import map_to_location, MappingError


class ScanProcessingError(Exception):
    """Service-level error that the router converts into an HTTP error."""

    def __init__(self, status_code: int, detail: str):
        super().__init__(detail)
        self.status_code = status_code
        self.detail = detail


DUPLICATE_WINDOW_SECONDS = 10


def process_scan(
    pallet_id: str,
    x: float,
    y: float,
    z: float,
    confidence: float,
):
    # Open one DB connection for this scan-processing request.
    connect = get_connection()

    try:
        # Convert physical drone coordinates into logical warehouse location.
        try:
            mapped_aisle, mapped_bay, mapped_level = map_to_location(x, y, z)
        except MappingError as e:
            raise ScanProcessingError(status_code=422, detail=str(e))

        # If barcode is missing, store an exception event instead of a normal scan row.
        if not pallet_id or pallet_id.strip() == "":
            cursor = connect.execute(
                """
                INSERT INTO Exceptions (palletID, aisle, bay, level, reason)
                VALUES (?, ?, ?, ?, ?)
                """,
                ("", str(mapped_aisle), str(mapped_bay), mapped_level, "BARCODE_NOT_FOUND"),
            )
            exception_id = cursor.lastrowid
            connect.commit()
            raise ScanProcessingError(
                status_code=422,
                detail=f"Barcode not detected. Exception recorded with exceptionID {exception_id}.",
            )

        # Simple dedupe heuristic for current project phase:
        # if same pallet/location appears within a short time window, return existing row.
        existing_row = connect.execute(
            """
            SELECT * FROM Scan
            WHERE palletID = ?
              AND aisle = ?
              AND bay = ?
              AND level = ?
              AND timestamp >= datetime('now', ?)
            ORDER BY scanID DESC
            LIMIT 1
            """,
            (
                pallet_id,
                str(mapped_aisle),
                str(mapped_bay),
                mapped_level,
                f"-{DUPLICATE_WINDOW_SECONDS} seconds",
            ),
        ).fetchone()

        if existing_row is not None:
            return {
                "scanID": existing_row["scanID"],
                "palletID": pallet_id,
                "aisle": mapped_aisle,
                "bay": mapped_bay,
                "level": mapped_level,
                "x": x,
                "y": y,
                "z": z,
                "confidence": existing_row["confidence"],
                "duplicate": True,
            }

        # Normal scan: save mapped location and confidence to Scan table.
        confidence_value = float(confidence)
        cursor = connect.execute(
            "INSERT INTO Scan (palletID, aisle, bay, level, confidence) VALUES (?, ?, ?, ?, ?)",
            (pallet_id, str(mapped_aisle), str(mapped_bay), mapped_level, confidence_value),
        )
        scan_id = cursor.lastrowid
        connect.commit()

        # Return both mapped location and raw xyz for debugging purposes
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
    finally:
        connect.close()

