#!/usr/bin/env python3
import argparse
from html import parser
import os
import sys
import signal
import struct
import math
from pathlib import Path
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

try:
    from sensor_msgs_py import point_cloud2
except Exception as e:
    print("sensor_msgs_py.point_cloud2 not available. Source your ROS 2 environment.", file=sys.stderr)
    raise

# Optional TF transform for alignment
try:
    from tf2_ros import Buffer, TransformListener
    import tf_transformations
except Exception:
    Buffer = None
    TransformListener = None
    tf_transformations = None


def ensure_dir(path: Path):
    if path.parent and not path.parent.exists():
        path.parent.mkdir(parents=True, exist_ok=True)


def write_pcd_ascii(path: Path, xyz_i):
    """
    xyz_i: list/iterable of tuples (x,y,z, intensity) OR (x,y,z) if intensity is None
    """
    has_i = False
    for p in xyz_i:
        has_i = (len(p) >= 4)
        break

    N = len(xyz_i)
    if N == 0:
        raise RuntimeError("No points to save")

    header = []
    header.append("VERSION .7")
    if has_i:
        header.append("FIELDS x y z intensity")
        header.append("SIZE 4 4 4 4")
        header.append("TYPE F F F F")
        header.append("COUNT 1 1 1 1")
        header.append(f"WIDTH {N}")
        header.append("HEIGHT 1")
        header.append("VIEWPOINT 0 0 0 1 0 0 0")
        header.append(f"POINTS {N}")
        header.append("DATA ascii")
    else:
        header.append("FIELDS x y z")
        header.append("SIZE 4 4 4")
        header.append("TYPE F F F")
        header.append("COUNT 1 1 1")
        header.append(f"WIDTH {N}")
        header.append("HEIGHT 1")
        header.append("VIEWPOINT 0 0 0 1 0 0 0")
        header.append(f"POINTS {N}")
        header.append("DATA ascii")

    with path.open("w") as f:
        f.write("\n".join(header) + "\n")
        if has_i:
            for x, y, z, i in xyz_i:
                f.write(f"{x} {y} {z} {i}\n")
        else:
            for x, y, z in xyz_i:
                f.write(f"{x} {y} {z}\n")


def write_pcd_binary(path: Path, xyz_i):
    has_i = False
    for p in xyz_i:
        has_i = (len(p) >= 4)
        break

    N = len(xyz_i)
    if N == 0:
        raise RuntimeError("No points to save")

    if has_i:
        header = (
            "VERSION .7\n"
            "FIELDS x y z intensity\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "COUNT 1 1 1 1\n"
            f"WIDTH {N}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {N}\n"
            "DATA binary\n"
        )
        pack_fmt = "<ffff"
    else:
        header = (
            "VERSION .7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {N}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {N}\n"
            "DATA binary\n"
        )
        pack_fmt = "<fff"

    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        for p in xyz_i:
            if has_i:
                x, y, z, i = p
                f.write(struct.pack(pack_fmt, float(x), float(y), float(z), float(i)))
            else:
                x, y, z = p[:3]
                f.write(struct.pack(pack_fmt, float(x), float(y), float(z)))


def voxel_downsample(points, leaf):
    """Naive voxel downsample: hash by floor(coord/leaf)."""
    if leaf <= 0.0 or len(points) == 0:
        return points
    vox = {}
    inv = 1.0 / float(leaf)
    for p in points:
        x, y, z = p[:3]
        key = (int(math.floor(x * inv)),
               int(math.floor(y * inv)),
               int(math.floor(z * inv)))
        # Keep last sample per voxel (fast & simple)
        vox[key] = p
    return list(vox.values())


class PCDRecorder(Node):
    def __init__(self, args):
        super().__init__("pcd_recorder")

        self.input_topic = args.topic
        self.save_path = Path(args.save)
        self.binary = args.binary
        self.max_points = args.max_points
        self.voxel_leaf = args.voxel_leaf
        self.target_frame = args.target_frame
        self.frame_skip = max(1, args.frame_skip)
        self._frame_counter = 0

        self._lock = Lock()
        self._points = []  # list of tuples (x,y,z[,i])

        # QoS: SensorData for LiDAR-like topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT if args.best_effort else ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.get_logger().info(f"Subscribing to {self.input_topic}")
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, qos)

        # TF for optional alignment
        if self.target_frame and Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info(f"Will transform clouds to target frame: {self.target_frame}")
        else:
            self.tf_buffer = None
            self.tf_listener = None
            if self.target_frame:
                self.get_logger().warn("tf2_ros not available in Python; will append without transform.")

        # Services
        self.save_srv = self.create_service(Trigger, "save_map", self.on_save)
        self.reset_srv = self.create_service(Trigger, "reset_map", self.on_reset)

        self.get_logger().info(
            f"Accumulating clouds → {'binary' if self.binary else 'ascii'} PCD at {self.save_path}"
        )
        if self.voxel_leaf > 0.0:
            self.get_logger().info(f"Voxel downsample enabled: leaf={self.voxel_leaf} m")
        self.get_logger().info(f"Max points: {self.max_points}")

    def on_cloud(self, msg: PointCloud2):
        # Extract fields
        self._frame_counter += 1
        if (self._frame_counter % self.frame_skip) != 0:
            return
        has_intensity = any(f.name == "intensity" for f in msg.fields)
        use_tf = self.target_frame and self.tf_buffer is not None

        # If using TF, try to get transform
        T = None
        if use_tf:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,  # target
                    msg.header.frame_id or "",  # source
                    msg.header.stamp
                )
                # Convert geometry_msgs Transform to 4x4
                t = tf.transform.translation
                q = tf.transform.rotation
                import numpy as np
                from math import isfinite
                # 4x4 from translation + quaternion
                R = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
                R[0, 3] = t.x; R[1, 3] = t.y; R[2, 3] = t.z
                T = R
            except Exception as e:
                self.get_logger().warn_throttle(2.0, f"No TF {msg.header.frame_id}->{self.target_frame}: {e}; appending as-is")
                T = None

        # Read points
        it = point_cloud2.read_points(msg, field_names=None, skip_nans=True)
        new_pts = []
        if T is None:
            if has_intensity:
                for p in it:
                    # p tuple may be (x,y,z, intensity, …) → first 4 only
                    new_pts.append((float(p[0]), float(p[1]), float(p[2]), float(p[3])))
            else:
                for p in it:
                    new_pts.append((float(p[0]), float(p[1]), float(p[2])))
        else:
            import numpy as np
            for p in it:
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                v = np.array([x, y, z, 1.0], dtype=float)
                vt = T @ v
                if has_intensity:
                    new_pts.append((float(vt[0]), float(vt[1]), float(vt[2]), float(p[3])))
                else:
                    new_pts.append((float(vt[0]), float(vt[1]), float(vt[2])))

        with self._lock:
            self._points.extend(new_pts)
            # Optional voxel
            if self.voxel_leaf > 0.0:
                self._points = voxel_downsample(self._points, self.voxel_leaf)
            # Clamp size
            if len(self._points) > self.max_points:
                # keep the most recent max_points
                self._points = self._points[-self.max_points:]

    def on_save(self, req, res):
        try:
            ensure_dir(self.save_path)
            with self._lock:
                pts = list(self._points)
            if len(pts) == 0:
                res.success = False
                res.message = "No points accumulated yet."
                return res
            if self.binary:
                write_pcd_binary(self.save_path, pts)
            else:
                write_pcd_ascii(self.save_path, pts)
            res.success = True
            res.message = f"Saved {len(pts)} points to {self.save_path}"
        except Exception as e:
            res.success = False
            res.message = f"Save failed: {e}"
        return res

    def on_reset(self, req, res):
        with self._lock:
            self._points.clear()
        res.success = True
        res.message = "Accumulation reset."
        return res


def main():
    parser = argparse.ArgumentParser(description="Accumulate a PointCloud2 topic and save to PCD.")
    parser.add_argument("--topic", required=True, help="Input PointCloud2 topic (e.g., /fast_limo/pointcloud)")
    parser.add_argument("--save", required=True, help="Output .pcd path (e.g., /home/sage/maps/run001.pcd)")
    parser.add_argument("--binary", action="store_true", help="Write binary PCD (default: ASCII)")
    parser.add_argument("--frame-skip", type=int, default=1)
    parser.add_argument("--voxel-leaf", type=float, default=0.0, help="Optional voxel size in meters (0 disables)")
    parser.add_argument("--max-points", type=int, default=20000000, help="Max points to keep in memory")
    parser.add_argument("--target-frame", type=str, default="", help="If set, transform clouds into this TF frame")
    parser.add_argument("--best-effort", action="store_true", help="Use BEST_EFFORT QoS for input (default RELIABLE)")
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = PCDRecorder(args)

    # Save on Ctrl-C too:
    def _sigint(_sig, _frm):
        try:
            req = Trigger.Request()
            res = node.on_save(req, Trigger.Response())
            node.get_logger().info(res.message)
        finally:
            rclpy.shutdown()
            sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
