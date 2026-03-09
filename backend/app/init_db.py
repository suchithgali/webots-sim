from app.db import get_connection


# Create the Scan table and related indexes
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
    connect.execute(create_index_pallet_sql)
    connect.execute(create_index_location_sql)
    connect.commit()
    connect.close()


if __name__ == "__main__":
    init_db()
