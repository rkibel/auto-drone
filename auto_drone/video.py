from __future__ import annotations

import os
import subprocess
from glob import glob


def write_video(frame_dir: str, output_path: str, fps: int = 12) -> bool:
    """Encode rendered PNG frames into an MP4 with ffmpeg when available."""
    frame_pattern = os.path.join(frame_dir, "frame_*.png")
    if not glob(frame_pattern):
        return False

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    command = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(fps),
        "-pattern_type",
        "glob",
        "-i",
        frame_pattern,
        "-vf",
        "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        output_path,
    ]
    try:
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (FileNotFoundError, subprocess.CalledProcessError):
        return False
    return True


def clear_previous_outputs(frame_dir: str, video_path: str) -> None:
    for path in glob(os.path.join(frame_dir, "frame_*.png")):
        os.remove(path)
    final_path = os.path.join(frame_dir, "final.png")
    if os.path.exists(final_path):
        os.remove(final_path)
    if video_path and os.path.exists(video_path):
        os.remove(video_path)
