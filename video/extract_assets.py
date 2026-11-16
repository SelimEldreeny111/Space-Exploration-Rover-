"""Re-create video/assets/ from the original 2023 project folder.

The raw media are not stored in the repository: they are large, and the 2023
presentation also contains third-party material (a Perseverance animation,
stock photos, a newspaper clipping) that must not be redistributed and that
the video does not use. Only the team's own files below are extracted.

Usage:  python extract_assets.py "<path to 'Space Exploration Rover' folder>"
        (default: ../../Space Exploration Rover, i.e. the original layout)

make_video.py additionally reads Documentation/final presentation/
navigation.mp4 and sensor.mp4 directly from that folder.
"""
from __future__ import annotations

import os
import shutil
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT = os.path.join(os.path.dirname(os.path.dirname(HERE)), "Space Exploration Rover")

# file inside "All in One.pptx" -> name used by make_video.py
FROM_PPTX = {
    "ppt/media/media3.mp4": "media3.mp4",    # rover mapping run (team footage)
    "ppt/media/media4.mp4": "media4.mp4",    # robotic arm (team footage)
    "ppt/media/image42.png": "image42.png",  # Hector SLAM map on the laptop
    "ppt/media/image44.png": "image44.png",  # solar panel
    "ppt/media/image45.png": "image45.png",  # battery packs
}
FROM_SHOW = {"1.jpg": "s1.jpg", "2.jpg": "s2.jpg"}  # team photos


def main() -> None:
    src = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    doc = os.path.join(src, "Documentation")
    out = os.path.join(HERE, "assets", "pptx")
    os.makedirs(out, exist_ok=True)
    with zipfile.ZipFile(os.path.join(doc, "final presentation", "All in One.pptx")) as z:
        for member, name in FROM_PPTX.items():
            with z.open(member) as fi, open(os.path.join(out, name), "wb") as fo:
                shutil.copyfileobj(fi, fo)
            print("extracted", name)
    for name, target in FROM_SHOW.items():
        shutil.copyfile(os.path.join(doc, "show", name), os.path.join(out, target))
        print("copied", target)


if __name__ == "__main__":
    main()
