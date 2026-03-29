from sqlmodel import Session

from app.db import engine
from app.models import Scan


def seed():
    sample_scans = [
        ("PAL001", "A1", "B2", 1, 0.98),
        ("PAL002", "A2", "B3", 2, 0.95),
        ("PAL003", "B1", "C1", 0, 0.99),
        ("PAL004", "C3", "D4", 3, 0.87),
        ("PAL005", "A1", "B1", 1, 0.92),
    ]

    with Session(engine) as session:
        for pallet_id, aisle, bay, level, confidence in sample_scans:
            session.add(
                Scan(
                    palletID=pallet_id,
                    aisle=aisle,
                    bay=bay,
                    level=level,
                    confidence=confidence,
                )
            )
        session.commit()

    print(f"Seeded {len(sample_scans)} scans successfully")


if __name__ == "__main__":
    seed()
