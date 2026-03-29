from datetime import datetime
from typing import Optional

from sqlalchemy import Column, DateTime, Index, Text, text
from sqlmodel import Field, SQLModel


class Scan(SQLModel, table=True):
    __tablename__ = "Scan"
    __table_args__ = (
        Index("idx_scan_pallet", "palletID"),
        Index("idx_scan_location", "aisle", "bay", "level"),
    )

    scanID: Optional[int] = Field(default=None, primary_key=True)
    palletID: str = Field(sa_type=Text)
    aisle: str = Field(sa_type=Text)
    bay: str = Field(sa_type=Text)
    level: int
    confidence: float = 1.0
    timestamp: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime, server_default=text("CURRENT_TIMESTAMP")),
    )


class WarehouseException(SQLModel, table=True):
    __tablename__ = "Exceptions"

    exceptionID: Optional[int] = Field(default=None, primary_key=True)
    palletID: Optional[str] = Field(default=None, sa_type=Text)
    aisle: str = Field(sa_type=Text)
    bay: str = Field(sa_type=Text)
    level: int
    reason: str = Field(sa_type=Text)
    timestamp: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime, server_default=text("CURRENT_TIMESTAMP")),
    )


class Skeleton(SQLModel, table=True):
    __tablename__ = "Skeleton"

    scanID: Optional[int] = Field(default=None, primary_key=True)
    timestamp: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime, server_default=text("CURRENT_TIMESTAMP")),
    )
    barcodes: str = Field(sa_type=Text)
