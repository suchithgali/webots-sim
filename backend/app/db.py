import sqlite3, json
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DB_NAME = os.path.join(BASE_DIR, "warehouse.db")

def get_connection():
    connection = sqlite3.connect(DB_NAME)
    connection.row_factory = sqlite3.Row
    return connection

def insert_scan(barcodes_list):
    db = get_connection()
    barcodes_json = json.dumps(barcodes_list)
    db.execute(
        "INSERT INTO Skeleton (barcodes) VALUES (?)",
        (barcodes_json,)
    )
    db.commit()
    db.close()