import json
import time
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading

# Shared telemetry data dictionary
telemetry_data = {}
telemetry_file = "/Users/suchithgali/C++ Files/CSE120/S26-CSE-303/simulator/telemetry.json"

# HTTP request handler
class TelemetryHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        body = json.dumps(telemetry_data).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")  # Allow CORS
        self.end_headers()
        self.wfile.write(body)
    
    def log_message(self, *args):
        pass  # Suppress request logging

# Background thread to continuously read telemetry file
def update_telemetry():
    global telemetry_data
    while True:
        try:
            if os.path.exists(telemetry_file):
                with open(telemetry_file, "r") as f:
                    telemetry_data = json.load(f)
        except Exception:
            pass  # Ignore errors 
        time.sleep(0.05)  # Update at 20 Hz

# Start the update thread
update_thread = threading.Thread(target=update_telemetry, daemon=True)
update_thread.start()

# Start HTTP server
print(f"Reading telemetry from: {telemetry_file}")
print("Serving telemetry at http://localhost:8765")
print("Press Ctrl+C to stop")

try:
    server = HTTPServer(("", 8765), TelemetryHandler)
    server.serve_forever()
except KeyboardInterrupt:
    print("\nServer stopped")
