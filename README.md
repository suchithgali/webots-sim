<div align="center">

# Autonomous Inventory Drone for Pallet Location Verification (After-Hours Scan + Map + Exceptions List)

Project Description: Warehouses rely on operators scanning pallet identifiers when moving products to a new location. When a pallet is moved but the drop-off scan is missed, the WMS still shows the old location — leading to time wasted searching, picking delays, and “lost pallet” investigations

</div>


## Overview

United States Cold Storage (USCS) operates large-scale temperature-controlled warehouses across the United States, managing over 376 million cubic feet of refrigerated distribution space. Accurate pallet tracking is essential to maintaining efficient warehouse operations. When pallet relocations are not properly scanned, the Warehouse Management System (WMS) retains incorrect location data, leading to lost inventory searches, picking delays, and operational inefficiencies. This project develops an autonomous indoor drone capable of performing after-hours warehouse audits by scanning pallet identifiers, mapping storage locations, and automatically identifying discrepancies between physical inventory and WMS records.

### Goals

- [ ] Autonomously patrol warehouse aisles after operating hours
- [ ] Detect and track problem zones with frequent inventory errors
- [ ] Scan pallet identifiers without human intervention
- [ ] Compare scanned results against WMS expectations
- [ ]  Generate automated discrepancy and analytics reports
- [ ]   Reduce manual inventory verification labor

### Features

- [ ] Drone payload + sensing package selection (camera/illumination, depth/LiDAR)
- [ ] Navigation + mapping pipeline (SLAM/waypoint patrol routes) with rack/aisle localization
- [ ] Identifier scanning module (barcode, confidence scoring, duplicate handling)
- [ ] Reconciliation report: “Found vs Expected” + exception list + heatmap of problem zones
- [ ] Dashboard view showing scanned pallets on a warehouse map and trend tracking over time

### Software Stack / Technologies Used

- Robotics: Barcode reader, Webots
- Frontend: HTML/CSS/JS
- Backend: FastAPI (Python)
- Database: SQLite
- PI: REST over HTTP (JSON)
- Export: CSV/XLSX generator (pandas)
- Simulation: Webots
## Quickstart

Summary for developers with links to setup, build, test instructions in wiki or docs.
- information pending 

## Structure

Include: what constitutes passing (e.g., all tests green, coverage threshold).
- information pending

## Team Members and Roles
Suchith Gali - Fullstack Engineer/Technical Lead
Laith Darras - Backend & DevOps Engineer

## Coding & Collaboration Conventions

- Use semantic commit messages (see `CONTRIBUTING.md` for full details).
- Open an Issue for every distinct unit of work (lab task, feature, bug, refactor, research).
- Create branches from `main` named after the Issue: `<type>/short-kebab` (e.g., `feat/scheduler-phase1`).
- Commit changes incrementally with semantic commit messages.
- Open a Pull Request early (draft) and link the Issue.
- Request peer review (if required) before merging.
- Squash merge or rebase to keep `main` linear (unless told otherwise).
