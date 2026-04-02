# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys
import os
import platform
import subprocess
import time

import webots_setup
from utils import get_webots_path
from drone_controller import Mavic

if __name__ == "__main__":
    simulator_dir = os.path.dirname(os.path.abspath(__file__))

    # Webots sets this env variable when it launches a controller —
    # if it's present we're already inside Webots, so skip the subprocess launch
    already_in_webots = (
        "WEBOTS_ROBOT_ID"        in os.environ or
        "WEBOTS_CONTROLLER_URL"  in os.environ
    )

    webots_process = None
    if not already_in_webots:
        webots_exe = get_webots_path()
        wbt_path   = os.path.join(simulator_dir, "world", "worlds", "uscs_cold_storage.wbt")

        if webots_exe and os.path.exists(webots_exe):
            print(f"[Launcher] Starting Webots: {webots_exe}", flush=True)
            launch_env = os.environ.copy()
            launch_env.pop("DYLD_LIBRARY_PATH", None)  # macOS-only, safe to remove on Windows
            webots_process = subprocess.Popen([webots_exe, wbt_path], env=launch_env)
        else:
            print("[Launcher] Webots not found — assuming it launched this script.", flush=True)

    try:
        robot = Mavic()
        robot.run()
    except KeyboardInterrupt:
        print("\n[Launcher] Manual interrupt received. Stopping patrol early.", flush=True)
    
    # Run the database update regardless of whether we finished naturally or Ctrl+C
    print(f"[Launcher] Preparing to save {len(robot.scanned_codes)} scanned barcodes to database...", flush=True)
    backend_path = os.path.join(simulator_dir, "..", "backend")
    if backend_path not in sys.path:
        sys.path.append(backend_path)
        
    # Dynamically inject the backend virtual environment's site-packages so the simulator can use sqlalchemy/sqlmodel
    import glob
    venv_site_packages = glob.glob(os.path.join(backend_path, ".venv", "lib", "python*", "site-packages"))
    if venv_site_packages and venv_site_packages[0] not in sys.path:
        sys.path.append(venv_site_packages[0])
    
    try:
        print("[Launcher] Translating Webots coordinates to JSON inches...", flush=True)
        from app.services.scan_service import process_scan
        import json
        
        # Load the JSON directly to do the math
        config_path = os.path.join(simulator_dir, "..", "backend", "app", "configs", "warehouses", "default.json")
        with open(config_path, "r") as f:
            config = json.load(f)

        success_count = 0
        for scan in robot.scanned_detail:
            barcode = scan["barcode"]
            sim_x = scan["sim_x"]
            sim_y = scan["sim_y"]
            sim_z = scan["sim_z"]
            
            # In Webots, the bottom rack is around 1.0m and the top rack is around 2.22m.
            # We map a drone height of < 1.5 meters to 36.0 real-world inches (Level 1).
            # We map any drone height >= 1.5 meters to 108.0 real-world inches (Level 2).
            rw_z = 36.0 if sim_z < 1.5 else 108.0
            
            # Webots Y-coordinate corresponds to the real-world X-coordinate (aisle width).
            # The first aisle starts at Y = -10.70 in the simulator.
            # Each subsequent aisle is spaced exactly 3.35 meters apart.
            # By shifting (+10.70) and dividing by 3.35, we get a 0-based aisle index!
            aisle_idx = int(round((sim_y + 10.70) / 3.35))
            aisles = config.get("aisles", [])
            # If the calculated index is valid (within the list of configured aisles)
            if 0 <= aisle_idx < len(aisles):
                # Calculate the exact center of the real-world aisle by averaging its start and end X values
                rw_x = (aisles[aisle_idx]["x_start"] + aisles[aisle_idx]["x_end"]) / 2.0
            else:
                # Fallback: if somehow out of bounds, snap to the last known aisle's endpoint or 0.0
                rw_x = aisles[-1]["x_end"] if aisle_idx > 0 else 0.0

            # Webots X-coordinate corresponds to the real-world Y-coordinate (length down the aisle).
            # Grab the warehouse's starting Y boundary and calculate total length (span) in inches.
            rw_y_start = config["y_bounds"]["start"]
            rw_y_span  = config["y_bounds"]["end"] - rw_y_start
            
            # The drone scans in a range from X = -5.4 to X = 5.4 in Webots.
            # Normalize the drone's X position to a percentage (0.0 at start, 1.0 at end).
            frac = (sim_x - (-5.4)) / (5.4 - (-5.4))
            # Clamp the percentage between 0 and 1 so flight overshoot doesn't cause negative inches.
            frac = max(0.0, min(1.0, frac))
            # Multiply the percentage by the total inches of the warehouse to get the exact inch position.
            rw_y = rw_y_start + frac * rw_y_span

            try:
                process_scan(
                    warehouse_id="default",
                    pallet_id=barcode,
                    x=rw_x,
                    y=rw_y,
                    z=rw_z,
                    confidence=1.0
                )
                success_count += 1
            except Exception as err:
                print(f"Failed to process scan for {barcode}: {err}", flush=True)

        print(f"[Launcher] Database updated successfully with {success_count} exact pallet locations to Scan!")
    except Exception as e:
        print(f"[Launcher] Skipping database update: failed to load backend logic or db map: {e}", flush=True)

    finally:
        if webots_process:
            print("[Launcher] Closing Webots...", flush=True)
            webots_process.terminate()
            try:
                webots_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                webots_process.kill()
            print("[Launcher] Done.")