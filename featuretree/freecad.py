"""freecad.py — locate FreeCAD's headless `freecadcmd` (its own Python 3.11).

Like the wirebender cell, FreeCAD is driven through its OWN interpreter. We find
(or extract once, cached) the AppImage's freecadcmd and run our emitter/reader
scripts under it. Override with FREECAD_CMD or FREECAD_APPIMAGE.
"""

import os
import subprocess
from pathlib import Path

DEFAULT_APPIMAGE = "/opt/FreeCAD_1.0.2-conda-Linux-x86_64-py311.AppImage"
CACHE = Path.home() / ".cache" / "software-mfg" / "freecad"


def freecadcmd_path() -> str:
    env = os.environ.get("FREECAD_CMD")
    if env and Path(env).exists():
        return env
    for p in ("/tmp/squashfs-root/usr/bin/freecadcmd",
              str(CACHE / "squashfs-root/usr/bin/freecadcmd")):
        if Path(p).exists():
            return p
    appimage = os.environ.get("FREECAD_APPIMAGE", DEFAULT_APPIMAGE)
    if not Path(appimage).exists():
        raise FileNotFoundError(f"FreeCAD AppImage not found: {appimage} (set FREECAD_APPIMAGE)")
    CACHE.mkdir(parents=True, exist_ok=True)
    subprocess.run([appimage, "--appimage-extract"], cwd=str(CACHE), check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return str(CACHE / "squashfs-root/usr/bin/freecadcmd")


def run_in_freecad(script: str, env_vars=None, capture=True):
    """Run a script under freecadcmd. Pass data via env_vars, NOT argv — freecadcmd
    treats extra path args as documents to open, not script arguments."""
    cmd = [freecadcmd_path(), script]
    env = {**os.environ, "QT_QPA_PLATFORM": "offscreen"}
    if env_vars:
        env.update({k: str(v) for k, v in env_vars.items()})
    return subprocess.run(cmd, capture_output=capture, text=True, env=env)
