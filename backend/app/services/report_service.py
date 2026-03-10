import csv
import openpyxl
from app.db import get_connection


# Export all scan records to an Excel sheet
def export_scans_xlsx(path="scans.xlsx"):
    connect = get_connection()
    rows = connect.execute("SELECT * FROM Scan").fetchall()
    connect.close()

    workbook = openpyxl.Workbook()
    sheet = workbook.active
    sheet.title = "Scans"

    header = ["License Plate", "Location", "Timestamp"]
    sheet.append(header)

    sheet.column_dimensions["A"].width = 18
    sheet.column_dimensions["B"].width = 14
    sheet.column_dimensions["C"].width = 20

    for row in rows:
        license_plate = row["palletID"]
        location = f"{row['aisle']}-{row['bay']}-{row['level']}"
        timestamp = row["timestamp"]

        row_values = [license_plate, location, timestamp]
        sheet.append(row_values)

    workbook.save(path)

    return path


# Export all scan records to a CSV file
def export_scans_csv(path="scans.csv"):
    connect = get_connection()
    rows = connect.execute("SELECT * FROM Scan").fetchall()
    connect.close()

    with open(path, "w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)

        writer.writerow(["License Plate", "Location", "Timestamp"])

        for row in rows:
            license_plate = row["palletID"]
            location = f"{row['aisle']}-{row['bay']}-{row['level']}"
            timestamp = row["timestamp"]

            writer.writerow([license_plate, location, timestamp])

    return path

