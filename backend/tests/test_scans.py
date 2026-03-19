from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_create_scan():
    payload = {
        "palletID": "P1",
        "aisle": "01",
        "bay": "02",
        "level": 1,
        "confidence": 0.99,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 201

    data = response.json()
    assert data["palletID"] == payload["palletID"]
    assert data["aisle"] == payload["aisle"]
    assert data["bay"] == payload["bay"]
    assert data["level"] == payload["level"]
    assert data["confidence"] == payload["confidence"]


def reject_low_confidence():
    payload = {
        "palletID": "P1",
        "aisle": "01",
        "bay": "02",
        "level": 1,
        "confidence": 0.5,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422


def reject_negative_level():
    payload = {
        "palletID": "P1",
        "aisle": "01",
        "bay": "02",
        "level": -1,
        "confidence": 0.99,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422


def reject_negative_confidence():
    payload = {
        "palletID": "P1",
        "aisle": "01",
        "bay": "02",
        "level": 1,
        "confidence": -0.1,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422


def reject_confidence_above_one():
    payload = {
        "palletID": "P1",
        "aisle": "01",
        "bay": "02",
        "level": 1,
        "confidence": 1.1,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422


def reject_empty_aisle():
    payload = {
        "palletID": "P1",
        "aisle": "   ",
        "bay": "02",
        "level": 1,
        "confidence": 0.99,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422


def reject_empty_bay():
    payload = {
        "palletID": "P1",
        "aisle": "01",
        "bay": "",
        "level": 1,
        "confidence": 0.99,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422