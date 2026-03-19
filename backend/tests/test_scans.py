from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_create_scan():
    payload = {
        "palletID": "P1",
        "aisle": "01",
        "bay": "02",
        "level": 1,
        "confidence": 0.9,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 201

    data = response.json()
    assert data["palletID"] == payload["palletID"]
    assert data["aisle"] == payload["aisle"]
    assert data["bay"] == payload["bay"]
    assert data["level"] == payload["level"]
    assert data["confidence"] == payload["confidence"]