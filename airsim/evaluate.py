#!/usr/bin/env python3
"""Scores the programs' estimates against AirSim's ground truth, from a
rosbag2 recording (airsim.launch.py record:=DIR), with evo.

    evaluate.py BAG [--truth /tf:enu.drone] [--skip S] NAME=TOPIC ...

    BAG       the recording (the projects' run scripts make one per run)
    --truth   the true trajectory: by default the TF enu -> drone, which
              AirSim's ROS node publishes from its odometry (airsim.launch.py
              adds enu)
    --skip    leave out the first S seconds of the recording (take-off)
    NAME=TOPIC  an estimate to score: a geometry_msgs/PoseStamped or
              nav_msgs/Odometry topic in the enu frame, each message stamped
              with the time it describes (a prediction one second ahead is
              stamped one second after it was made)

The truth is interpolated at each estimate's stamps (evo's
PoseTrajectory3D.interpolate; estimates outside the truth's time span are
dropped), then evo's APE, translation part without any alignment, is the 3D
error of each estimate. Prints, in metres, mean, rms, median, 95th
percentile and max of the 3D error, the mean vertical error and the number
of samples.

For plots and more metrics use evo's own tools on the same bag, e.g.
    evo_traj bag2 BAG /pole/fused --ref /tf:enu.drone --plot
    evo_ape bag2 BAG /tf:enu.drone /positioning/prediction --plot
Runs in localization_core/airsim/.venv (airsim/setup.sh installs evo) with
ROS 2 sourced: evo reads TF from a recording with tf2_py.
"""
import argparse
import sys

import numpy as np
from evo.core import metrics
from evo.tools import file_interface
from rosbags.rosbag2 import Reader


def score(truth, estimate, skip):
    start = truth.timestamps[0] + skip
    keep = (estimate.timestamps >= start) & (estimate.timestamps <= truth.timestamps[-1])
    estimate.reduce_to_ids(np.flatnonzero(keep))
    if estimate.num_poses == 0:
        return None
    reference = truth.interpolate(estimate.timestamps)
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((reference, estimate))
    error = ape.error
    vertical = np.abs(estimate.positions_xyz[:, 2] - reference.positions_xyz[:, 2])
    return {
        "mean": float(np.mean(error)),
        "rms": float(np.sqrt(np.mean(error ** 2))),
        "p50": float(np.percentile(error, 50)),
        "p95": float(np.percentile(error, 95)),
        "max": float(np.max(error)),
        "vertical": float(np.mean(vertical)),
        "samples": int(estimate.num_poses),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag")
    parser.add_argument("estimates", nargs="+", metavar="NAME=TOPIC")
    parser.add_argument("--truth", default="/tf:enu.drone")
    parser.add_argument("--skip", type=float, default=0.0)
    args = parser.parse_args()

    with Reader(args.bag) as reader:
        truth = file_interface.read_bag_trajectory(reader, args.truth, cache_tf_tree=True)
        rows = []
        for item in args.estimates:
            name, _, topic = item.rpartition("=")
            name = name or topic
            try:
                estimate = file_interface.read_bag_trajectory(reader, topic)
            except file_interface.FileInterfaceException:  # not recorded, or no messages
                rows.append((name, None))
                continue
            rows.append((name, score(truth, estimate, args.skip)))

    columns = ["mean", "rms", "p50", "p95", "max", "vertical", "samples"]
    width = max(len(name) for name, _ in rows) + 2
    print(f"3D error against {args.truth}, metres ({args.bag})")
    print(" " * width + "".join(f"{c:>9}" for c in columns))
    for name, result in rows:
        if result is None:
            print(f"{name:<{width}}  no samples")
            continue
        cells = "".join(f"{result[c]:9.3f}" for c in columns[:-1])
        print(f"{name:<{width}}{cells}{result['samples']:9d}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
