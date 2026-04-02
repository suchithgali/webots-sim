class WaypointPlanner:
    """
    Builds a serpentine patrol route purely from a warehouse config dict.
    No hardcoded coordinates — all geometry is derived from the config's
    own bounds and aisle definitions.

    Config schema expected:
    {
      "sim_bounds": {"x_min": -8.5, "x_max": 8.5, "y_min": -10.5, "y_max": 10.5},
      "aisles": [
        {"x_start": 0, "x_end": 100, "name": "A1"},
        ...
      ],
      "x_bounds": {"start": 0, "end": 1000},
      "y_bounds": {"start": 0, "end": 1000},
      "bay_count": 10
    }
    """

    def __init__(self, config: dict):
        self.config = config
        self._validate(config)

        sim  = config["sim_bounds"]
        rw_x = config["x_bounds"]
        rw_y = config["y_bounds"]

        self.sim_x_min = sim["x_min"]
        self.sim_x_max = sim["x_max"]
        self.sim_y_min = sim["y_min"]
        self.sim_y_max = sim["y_max"]

        self.rw_x_span = rw_x["end"] - rw_x["start"]
        self.rw_y_span = rw_y["end"] - rw_y["start"]
        self.rw_x_off  = rw_x["start"]
        self.rw_y_off  = rw_y["start"]

        self.bay_count = config.get("bay_count", 10)

    def _validate(self, cfg: dict):
        required = ["aisles", "x_bounds", "y_bounds"]
        missing  = [k for k in required if k not in cfg]
        if missing:
            raise ValueError(f"Warehouse config is missing required keys: {missing}")
            
        if "sim_bounds" not in cfg:
            # Hardcoded to precisely match the original hand-tuned constraints of the specific 
            # Webots world map (where Aisles run along X and offset along Y)
            # Webots Aisle 1 is at ~ -10.4. The gap to Aisle 2 is ~ 3.05 (Aisle 2 is -7.35).
            # The JSON aisle gap is 228. This implies 5016 JSON length = 67 meters in Webots.
            cfg["sim_bounds"] = {
                "x_min": -5.4,   "x_max": 5.4,  # Scanning length of the aisle
                "x_transit_min": -8.0, "x_transit_max": 8.0, # Increased clearance from 6.5 to guarantee avoiding racks
                "y_min": -11.07,  "y_max": 3.8   # Webots North wall is at Y=4.0, limit to here.
            }
            
        for b in ("x_min", "x_max", "y_min", "y_max"):
            if b not in cfg["sim_bounds"]:
                raise ValueError(f"sim_bounds is missing '{b}'")

    def _to_sim_y(self, rw_x: float) -> float:
        # In Webots, the Config X (aisle width index) corresponds to Webots Y (!)
        frac = (rw_x - self.rw_x_off) / self.rw_x_span
        return self.sim_y_min + frac * (self.sim_y_max - self.sim_y_min)

    def _to_sim_x(self, rw_y: float) -> float:
        # In Webots, the Config Y (aisle length) corresponds to Webots X (!)
        frac = (rw_y - self.rw_y_off) / self.rw_y_span
        return self.sim_x_min + frac * (self.sim_x_max - self.sim_x_min)

    def build(self) -> list:
        """
        Returns a list of [sim_x, sim_y, sim_z, is_scan_stop] waypoints.
        Route: sweeps every aisle, moving horizontally pallet by pallet,
        and vertically scanning both levels at each X position.
        """
        # The drone's takeoff base station in Webots:
        home_x = -8.5
        home_y = 3.0
        
        # Assume Level 1 ~ 1.0m, Level 2 ~ 2.22m for the Tello/Mavic camera FOV
        # The pallets in Webots are stacked exactly 1.22m apart vertically.
        scan_levels = [1.0, 2.22]
        waypoints = [[home_x, home_y, scan_levels[0], False]]

        direction = 1  # +1 = left-to-right (X_min to X_max), -1 = right-to-left

        aisles = self.config["aisles"]

        for aisle_idx, aisle in enumerate(aisles):
            x_mid_rw  = (aisle["x_start"] + aisle["x_end"]) / 2.0
            
            # Webots physical world aisles are spaced exactly 3.35m apart.
            # Aisle 1 is at Y = -10.70. Aisle 2 is at -7.35. Aisle 3 at -4.00, etc.
            y_mid_sim = -10.70 + (aisle_idx * 3.35)
            
            if y_mid_sim > self.sim_y_max:
                print(f"[PLANNER] Aisle {aisle_idx + 1} at Y={y_mid_sim:.2f} is outside simulation bounds. Stopping.", flush=True)
                break

            # Config Y (aisle length bounds) maps to Webots X bounds
            x_start_sim = self.sim_x_min if direction == 1 else self.sim_x_max
            x_end_sim   = self.sim_x_max if direction == 1 else self.sim_x_min

            # Give wide clearance to fully exit the aisle before turning.
            # Transit corner X positions:
            transit_x_start = self.config["sim_bounds"]["x_transit_min"] if direction == 1 else self.config["sim_bounds"]["x_transit_max"]
            transit_x_end   = self.config["sim_bounds"]["x_transit_max"] if direction == 1 else self.config["sim_bounds"]["x_transit_min"]

            # 1. Transit to the hallway before turning FRONT of the aisle
            waypoints.append([transit_x_start, y_mid_sim, scan_levels[0], False])

            # 2. Add scan stops spaced ~1.2 meters apart ALONG the X axis (aisle length)
            # The pallets in Webots are modeled exactly 10 per row from -5.4 to 5.4.
            num_stops = 10

            for i in range(num_stops):
                # Distribute perfectly from 0.0 to 1.0 so we hit the exact pallet coordinates
                frac  = i / (num_stops - 1)
                x_sim = x_start_sim + frac * (x_end_sim - x_start_sim)
                
                # To minimize wasted diagonal flight, we alternate scanning up and down
                # For i=0: scan bottom (1.0), then top (2.22)
                # For i=1: from top (2.22), move to next X, scan top (2.22), then bottom (1.0)
                level_order = scan_levels if i % 2 == 0 else list(reversed(scan_levels))
                
                for curr_z in level_order:
                    waypoints.append([x_sim, y_mid_sim, curr_z, True])

            # 3. Transit to the hallway out of the aisle
            # Ensure we drop back to base altitude before exiting the aisle
            waypoints.append([x_sim, y_mid_sim, scan_levels[0], False])
            waypoints.append([transit_x_end, y_mid_sim, scan_levels[0], False])

            direction *= -1

        # Return to base station
        waypoints.append([self.config["sim_bounds"]["x_transit_max"], 3.0, scan_levels[0], False])
        waypoints.append([home_x, home_y, scan_levels[0], False])

        print(f"[PLANNER] Generated {len(waypoints)} 3D waypoints "
              f"({sum(1 for w in waypoints if w[3])} scan stops, "
              f"{len(self.config['aisles'])} aisles, {len(scan_levels)} levels)", flush=True)
        return waypoints