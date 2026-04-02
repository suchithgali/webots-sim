import json
import time
import os
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading

# FIX 4: was hardcoded to "/Users/suchithgali/..." — crashes on any other machine.
# Now uses the same get_telemetry_path() utility that the rest of the project uses.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utils import get_telemetry_path

telemetry_data = {}
telemetry_file = get_telemetry_path()


class TelemetryHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        body = json.dumps(telemetry_data).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass  # suppress request logging


def update_telemetry():
    global telemetry_data
    while True:
        try:
            if os.path.exists(telemetry_file):
                with open(telemetry_file, "r") as f:
                    telemetry_data = json.load(f)
        except Exception:
            pass
        time.sleep(0.05)  # 20 Hz


update_thread = threading.Thread(target=update_telemetry, daemon=True)
update_thread.start()

print(f"Reading telemetry from: {telemetry_file}")
print("Serving telemetry at http://localhost:8765")
print("Press Ctrl+C to stop")

try:
    server = HTTPServer(("", 8765), TelemetryHandler)
    server.serve_forever()
except KeyboardInterrupt:
    print("\nServer stopped")