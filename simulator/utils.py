import os
import platform

def get_telemetry_path():
    """Returns a writable telemetry path next to this script on any OS."""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "telemetry.json")

def get_webots_path():
    """Returns the Webots executable path for the current OS."""
    system = platform.system()
    if system == "Darwin":
        return "/Applications/Webots.app/Contents/MacOS/webots"
    elif system == "Windows":
        candidates = [
            r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
            r"C:\Program Files (x86)\Webots\msys64\mingw64\bin\webots.exe",
        ]
        for path in candidates:
            if os.path.exists(path):
                return path
        return None
    else:
        # Linux
        return "/usr/local/bin/webots"

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)
