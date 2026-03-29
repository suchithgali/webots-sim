import json
from pathlib import Path

from sqlmodel import Session, create_engine

BASE_DIR = Path(__file__).resolve().parent
DB_NAME = str(BASE_DIR / "warehouse.db")

engine = create_engine(
    f"sqlite:///{DB_NAME}",
    connect_args={"check_same_thread": False},
)


def insert_scan(barcodes_list: object) -> None:
    from app.models import Skeleton

    with Session(engine) as session:
        session.add(Skeleton(barcodes=json.dumps(barcodes_list)))
        session.commit()
