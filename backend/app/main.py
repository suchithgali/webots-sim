from fastapi import FastAPI
from fastapi.responses import FileResponse
from app.scan_router import router as scan_router
from app.services.report_service import export_scans_xlsx, export_scans_csv

app = FastAPI(title="Scanner API")

@app.get("/")
def root():
    return {"message": "Backend API running"}

@app.get("/health")
def health_check():
    return {"status": "ok"}

@app.get("/reports/export")
def export_report():
    path = export_scans_xlsx()
    return FileResponse(
        path,
        media_type="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
        filename="scans.xlsx",
    )

@app.get("/reports/export-csv")
def export_report_csv():
    path = export_scans_csv()
    return FileResponse(
        path,
        media_type="text/csv",
        filename="scans.csv",
    )

app.include_router(scan_router)