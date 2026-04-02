import sys
import os
import platform

# --- Webots Early Python Path Extension ---
# If you run this file from the terminal (not through Webots), it needs to know
# where the Webots controller API is located so the import doesn't fail.
if "WEBOTS_HOME" not in os.environ:
    _sys = platform.system()
    if _sys == "Darwin":
        os.environ["WEBOTS_HOME"] = "/Applications/Webots.app"
        _webots_py_path = os.path.join(os.environ["WEBOTS_HOME"], "Contents", "lib", "controller", "python")
    elif _sys == "Windows":
        os.environ["WEBOTS_HOME"] = r"C:\Program Files\Webots"
        _webots_py_path = os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python")
    else:
        os.environ["WEBOTS_HOME"] = "/usr/local/webots"
        _webots_py_path = os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python")
        
    if os.path.exists(_webots_py_path) and _webots_py_path not in sys.path:
        sys.path.append(_webots_py_path)

# --- Cross-platform site-packages resolution ---
# Only needed on macOS where Homebrew installs packages outside the default path
if platform.system() == "Darwin":
    # manually add the Homebrew site-packages to the path
    # this ensures Python looks where 'pip3 install --break-system-packages' put them
    homebrew_site_packages = [
        '/opt/homebrew/lib/python3.11/site-packages', 
        '/usr/local/lib/python3.11/site-packages',
        os.path.expanduser('~/Library/Python/3.11/lib/python/site-packages')
    ]
    for path in homebrew_site_packages:
        if os.path.exists(path) and path not in sys.path:
            sys.path.append(path)

    # fix the ZBar dynamic library path for macOS
    if os.path.exists('/opt/homebrew/lib'):
        os.environ['DYLD_LIBRARY_PATH'] = '/opt/homebrew/lib'
    elif os.path.exists('/usr/local/lib'):
        os.environ['DYLD_LIBRARY_PATH'] = '/usr/local/lib'
