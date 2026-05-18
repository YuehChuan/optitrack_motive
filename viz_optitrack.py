#!/usr/bin/env python3

import time
import threading
from collections import deque
import numpy as np

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

from optitrack_vendor.NatNetClient import NatNetClient


# ============================================================
# Config
# ============================================================

TRACK_IDS = [1, 9]
VIS_HZ = 60.0

COLORS = {
    1: 0x00ff00,
    9: 0xff0000,
}

# ============================================================
# Meshcat
# ============================================================

vis = meshcat.Visualizer().open()
print("Meshcat URL:", vis.url())


# ============================================================
# Motive → Meshcat
# ============================================================

"""
先繞X軸 +90° 再以旋轉後的Z軸 繞 Z軸+90°
R_CONV=Rz*Rx

Rz = 
    [0,  -1,  0],
    [1,  0,  0],
    [0, 0,  1]

Rx = 
    [1,  0,  0],
    [0,  0,  -1],
    [0, 1,  0]

"""
R_CONV = np.array([
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0]
], dtype=float)



def quat_to_motive_euler(qw, qx, qy, qz):

    R = quat_to_rot((qw, qx, qy, qz))


    pitch = np.arctan2(R[2,1], R[2,2])   # X
    yaw   = np.arcsin(-R[2,0])           # Y
    roll  = np.arctan2(R[1,0], R[0,0])   # Z

    return pitch, yaw, roll

def rad2deg(rad):
    return rad * 180.0 / np.pi

def debug_motive(rb_id,p_motive,rot_motive):
    x, y, z = p_motive
    qw, qx, qy, qz = rot_motive[0],rot_motive[1],rot_motive[2],rot_motive[3]
    pitch_m, yaw_m, roll_m = quat_to_motive_euler(qw, qx, qy, qz)

    print(f"ID:{rb_id} "
        f"x:{x:.3f} y:{y:.3f} z:{z:.3f} "
        f"Motive Pitch(X):{rad2deg(pitch_m):.3f} "
        f"Motive Yaw(Y):{rad2deg(yaw_m):.3f} "
        f"Motive Roll(Z):{rad2deg(roll_m):.3f}"
    )


def quat_to_rot(q):
    qw, qx, qy, qz = q

    n = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n < 1e-12:
        return np.eye(3)

    qw /= n
    qx /= n
    qy /= n
    qz /= n

    return np.array([
        [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,     1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy]
    ])


def build_transform(pos, rot):
    qw,qx, qy, qz = rot

    R_motive = quat_to_rot((qw, qx, qy, qz))
    R_robot = R_CONV @ R_motive @ R_CONV.T
    p_robot = R_CONV @ np.array(pos)

    T = np.eye(4)
    T[:3, :3] = R_robot
    T[:3, 3] = p_robot

    return T, p_robot


# ============================================================
# Axis
# ============================================================

def add_axis(name, length=0.2, radius=0.005):

    vis[name]["x"].set_object(
        g.Cylinder(length, radius),
        g.MeshLambertMaterial(color=0xff0000)
    )
    vis[name]["x"].set_transform(
        tf.translation_matrix([length/2, 0, 0]) @
        tf.rotation_matrix(np.pi/2, [0, 0, 1])
    )

    vis[name]["y"].set_object(
        g.Cylinder(length, radius),
        g.MeshLambertMaterial(color=0x00ff00)
    )
    vis[name]["y"].set_transform(
        tf.translation_matrix([0, length/2, 0])
    )

    vis[name]["z"].set_object(
        g.Cylinder(length, radius),
        g.MeshLambertMaterial(color=0x0000ff)
    )
    vis[name]["z"].set_transform(
        tf.translation_matrix([0, 0, length/2]) @
        tf.rotation_matrix(np.pi/2, [1, 0, 0])
    )


# ============================================================
# Robot (LOCK-FREE + DEQUE)
# ============================================================

class Robot:
    def __init__(self, rb_id, color):

        self.id = rb_id
        self.color = color

        #只保留最新pose
        self.pose_buf = deque(maxlen=1)

        # trajectory
        self.traj = deque(maxlen=2000)
        self.traj_index = 0
        self.last_traj_pos = None

        self.tracked = False

        self._init_visual()

    def _init_visual(self):

        add_axis(f"rb{self.id}/frame")

        vis[f"rb{self.id}/center"].set_object(
            g.Sphere(0.01),
            g.MeshLambertMaterial(color=self.color)
        )

    # ========================================================
    # NatNet thread
    # ========================================================
    def update(self, pos, rot, tracking_valid):

        # =========================
        # REORDER from Motive position, quaternion
        #  lock-free append
        # =========================

        p_motive = np.array([
            pos[0],
            pos[2],
            -pos[1]
        ])
        rot_motive = (rot[3], rot[0], rot[2], -rot[1])

        debug_motive(self.id , p_motive, rot_motive)

        self.pose_buf.append((p_motive, rot_motive, tracking_valid))

        if tracking_valid:
            if not self.tracked:
                self.tracked = True
                print(self.id, "tracked")
        else:
            if self.tracked:
                self.tracked = False
                print(self.id, "lost")

    # ========================================================
    # Visualization thread
    # ========================================================
    def visualize(self):

        if not self.pose_buf:
            return

        pos, rot, tracking_valid = self.pose_buf[-1]

        if not tracking_valid:
            return

        T, p = build_transform(pos, rot)

        vis[f"rb{self.id}/frame"].set_transform(T)
        vis[f"rb{self.id}/center"].set_transform(
            tf.translation_matrix(p)
        )

        # trajectory
        if self.last_traj_pos is None or np.linalg.norm(p - self.last_traj_pos) > 0.01:
            self._add_traj(p)
            self.last_traj_pos = p

    def _add_traj(self, p):

        self.traj.append(p)

        name = f"rb{self.id}/traj/{self.traj_index}"

        vis[name].set_object(
            g.Sphere(0.003),
            g.MeshLambertMaterial(color=self.color)
        )

        vis[name].set_transform(
            tf.translation_matrix(p)
        )

        self.traj_index += 1


# ============================================================
# Global robots
# ============================================================

robots = {
    rb: Robot(rb, COLORS[rb])
    for rb in TRACK_IDS
}


# ============================================================
# NatNet callback
# ============================================================

def rigid_body_callback(rb_id, pos, rot):

    if rb_id not in robots:
        return

    robots[rb_id].update(pos, rot, True)


# ============================================================
# Visualization thread
# ============================================================

def vis_loop():

    dt = 1.0 / VIS_HZ

    while True:
        t0 = time.time()

        for rb in robots.values():
            rb.visualize()

        time.sleep(max(0, dt - (time.time() - t0)))


# ============================================================
# Main
# ============================================================

def main():

    client = NatNetClient()
    client.set_server_address("192.168.40.147")
    client.set_client_address("192.168.40.18")
    client.set_nat_net_version(4, 2)

    client.set_use_multicast(False)

    client.rigid_body_listener = rigid_body_callback

    print("Connecting...")
    client.run("d")

    # start visualization thread
    threading.Thread(target=vis_loop, daemon=True).start()

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
