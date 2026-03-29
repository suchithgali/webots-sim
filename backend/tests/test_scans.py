from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)


# all valid record
def test_create_scan():
    payload = {
        "palletID": "P1",
        "x": 10.0,
        "y": 10.0,
        "z": 10.0,
        "confidence": 0.99,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 201

    data = response.json()
    assert data["palletID"] == payload["palletID"]
    assert data["aisle"] == 1
    assert data["bay"] == 1
    assert data["level"] == 1
    assert data["confidence"] == payload["confidence"]


def test_reject_low_confidence():
    payload = {
        "palletID": "P1",
        "x": 10.0,
        "y": 10.0,
        "z": 10.0,
        "confidence": 0.5,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422


def test_reject_out_of_bounds_z():
    payload = {
        "palletID": "P1",
        "x": 10.0,
        "y": 10.0,
        "z": -1.0,
        "confidence": 0.99,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422


def test_reject_negative_confidence():
    payload = {
        "palletID": "P1",
        "x": 10.0,
        "y": 10.0,
        "z": 10.0,
        "confidence": -0.1,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422


def test_reject_confidence_above_one():
    payload = {
        "palletID": "P1",
        "x": 10.0,
        "y": 10.0,
        "z": 10.0,
        "confidence": 1.1,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422


def test_reject_out_of_bounds_x():
    payload = {
        "palletID": "P1",
        "x": -1.0,
        "y": 10.0,
        "z": 10.0,
        "confidence": 0.99,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422


# x in a configured aisle gap should fail deterministic aisle lookup
def test_reject_between_aisles():
    payload = {
        "palletID": "P1",
        "x": 150.0,
        "y": 10.0,
        "z": 10.0,
        "confidence": 0.99,
    }

    response = client.post("/scans/", json=payload)
    assert response.status_code == 422