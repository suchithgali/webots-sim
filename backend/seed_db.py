from app.db import get_connection

def seed():
    db = get_connection()

    sample_scans = [
        ("PAL001", "A1", "B2", 1, 0.98),
        ("PAL002", "A2", "B3", 2, 0.95),
        ("PAL003", "B1", "C1", 0, 0.99),
        ("PAL004", "C3", "D4", 3, 0.87),
        ("PAL005", "A1", "B1", 1, 0.92),
    ]

    db.executemany(
        "INSERT INTO Scan (palletID, aisle, bay, level, confidence) VALUES (?, ?, ?, ?, ?)",
        sample_scans
    )

    db.commit()
    db.close()

    print(f"Seeded {len(sample_scans)} scans successfully")

if __name__ == "__main__":
    seed()