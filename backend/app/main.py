from fastapi import FastAPI

app = FastAPI(title="Scanner API")

@app.get("/")
def root():
    return {"message": "Backend API running"}

@app.get("/health")
def health_check():
    return {"status": "ok"}