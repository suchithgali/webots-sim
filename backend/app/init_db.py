import os
import sys

# Automatically put backend directory in Python path
backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(backend_dir)

from app.db import get_connection

def init_db():
    create_scan_table_sql = """
    CREATE TABLE IF NOT EXISTS Scan (
        scanID INTEGER PRIMARY KEY AUTOINCREMENT,
        palletID TEXT NOT NULL,
        aisle TEXT NOT NULL,
        bay TEXT NOT NULL,
        level INTEGER NOT NULL,
        confidence REAL DEFAULT 1.0,
        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
    );
    """

    create_skeleton_table_sql = """
    CREATE TABLE IF NOT EXISTS Skeleton (
        scanID INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
        barcodes TEXT NOT NULL
    );
    """

    # --- ADD THIS ---
    create_exceptions_table_sql = """
    CREATE TABLE IF NOT EXISTS Exceptions (
        exceptionID INTEGER PRIMARY KEY AUTOINCREMENT,
        palletID TEXT,
        aisle TEXT NOT NULL,
        bay TEXT NOT NULL,
        level INTEGER NOT NULL,
        reason TEXT NOT NULL,
        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
    );
    """

    create_index_pallet_sql = """
    CREATE INDEX IF NOT EXISTS idx_scan_pallet
    ON Scan(palletID);
    """

    create_index_location_sql = """
    CREATE INDEX IF NOT EXISTS idx_scan_location
    ON Scan(aisle, bay, level);
    """

    connect = get_connection()
    connect.execute(create_scan_table_sql)
    connect.execute(create_skeleton_table_sql)
    connect.execute(create_exceptions_table_sql)  
    connect.execute(create_index_pallet_sql)
    connect.execute(create_index_location_sql)
    connect.commit()
    connect.close()


if __name__ == "__main__":
    init_db()