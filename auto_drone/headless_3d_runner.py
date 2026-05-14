from __future__ import annotations

import argparse
import os

from auto_drone.core3d import ActiveMappingSim3D
from auto_drone.render3d import render_sim3d
from auto_drone.video import clear_previous_outputs, write_video


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the headless 3D voxel active mapping simulator.")
    parser.add_argument("--width", type=int, default=32)
    parser.add_argument("--height", type=int, default=32)
    parser.add_argument("--depth", type=int, default=12)
    parser.add_argument("--obstacle-density", type=float, default=0.14)
    parser.add_argument("--sensor-radius", type=int, default=6)
    parser.add_argument("--sensor-fov-degrees", type=float, default=75.0)
    parser.add_argument("--sensor-noise", type=float, default=0.06)
    parser.add_argument("--max-steps", type=int, default=1500)
    parser.add_argument("--render-every", type=int, default=3)
    parser.add_argument("--output-dir", default="frames3d")
    parser.add_argument("--video-path", default="frames3d/exploration3d.mp4")
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--seed", type=int, default=11)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    clear_previous_outputs(args.output_dir, args.video_path)
    sim = ActiveMappingSim3D(
        width=args.width,
        height=args.height,
        depth=args.depth,
        obstacle_density=args.obstacle_density,
        sensor_radius=args.sensor_radius,
        sensor_fov_degrees=args.sensor_fov_degrees,
        sensor_noise=args.sensor_noise,
        seed=args.seed,
    )

    last = None
    stop_reason = None
    for _ in range(args.max_steps):
        last = sim.step()
        if last.step % args.render_every == 0 or last.done:
            render_sim3d(sim, os.path.join(args.output_dir, f"frame_{last.step:04d}.png"))
        print(
            f"step={last.step:04d} known={last.known_ratio:.3f} "
            f"uncertainty={last.mean_uncertainty:.3f} "
            f"pose=({last.pose.x},{last.pose.y},{last.pose.z},"
            f"{last.pose.roll:.2f},{last.pose.pitch:.2f},{last.pose.yaw:.2f}) "
            f"est=({last.estimated_pose.x},{last.estimated_pose.y},{last.estimated_pose.z},"
            f"{last.estimated_pose.roll:.2f},{last.estimated_pose.pitch:.2f},{last.estimated_pose.yaw:.2f}) "
            f"repr={last.mean_reprojection_error:.3f} keyframes={last.keyframe_count} "
            f"target={format_target(last.target)}"
        )
        if last.done:
            stop_reason = last.stop_reason
            break
    else:
        stop_reason = "max_steps_reached"

    if last is not None:
        render_sim3d(sim, os.path.join(args.output_dir, "final.png"))
        print(f"wrote frames to {os.path.abspath(args.output_dir)}")
        if write_video(args.output_dir, args.video_path, fps=args.fps):
            print(f"wrote video to {os.path.abspath(args.video_path)}")
        else:
            print("video was not written; install ffmpeg or inspect the PNG frames directly")
        print(f"stopped because: {stop_reason}")


def format_target(target) -> str:
    if target is None:
        return "None"
    return f"({target.x},{target.y},{target.z},{target.roll:.2f},{target.pitch:.2f},{target.yaw:.2f})"


if __name__ == "__main__":
    main()
