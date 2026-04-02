from enum import Enum

class DroneState(Enum):
    IDLE       = 0
    TAKEOFF    = 1
    NAVIGATING = 2
    CAPTURING  = 3
    LANDING    = 4
    RETURNING  = 5
    CHARGING   = 6
    PAUSED     = 7
    FAILURE    = 8
    COMPLETED  = 9
