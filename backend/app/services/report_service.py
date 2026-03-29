import csv

import openpyxl
from sqlmodel import Session, select

from app.db import engine
from app.models import Scan


def export_scans_xlsx(path="scans.xlsx"):
    with Session(engine) as session:
        rows = session.exec(select(Scan)).all()

    workbook = openpyxl.Workbook()
    sheet = workbook.active
    sheet.title = "Scans"

    header = ["License Plate", "Location", "Timestamp"]
    sheet.append(header)

    sheet.column_dimensions["A"].width = 18
    sheet.column_dimensions["B"].width = 14
    sheet.column_dimensions["C"].width = 20

    for row in rows:
        license_plate = row.palletID
        location = f"{row.aisle}-{row.bay}-{row.level}"
        timestamp = row.timestamp

        sheet.append([license_plate, location, timestamp])

    workbook.save(path)

    return path


def export_scans_csv(path="scans.csv"):
    with Session(engine) as session:
        rows = session.exec(select(Scan)).all()

    with open(path, "w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)

        writer.writerow(["License Plate", "Location", "Timestamp"])

        for row in rows:
            license_plate = row.palletID
            location = f"{row.aisle}-{row.bay}-{row.level}"
            timestamp = row.timestamp

            writer.writerow([license_plate, location, timestamp])

    return path
