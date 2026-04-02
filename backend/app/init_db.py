import os
import sys

from sqlmodel import SQLModel

from app.db import engine

# Import models so they register on SQLModel.metadata
from app.models import Scan, Skeleton, WarehouseException, TestScan  # noqa: F401

backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(backend_dir)


def init_db() -> None:
    SQLModel.metadata.create_all(engine)


if __name__ == "__main__":
    init_db()
