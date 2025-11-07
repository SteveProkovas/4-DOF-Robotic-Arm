"""
4DOF_Robotic_Arm_Simulation.py

Enhanced version (updated with user suggestions):
- Optimized common rotations helpers
- Robust get_joint_axis_world for CCD
- Optional history recording (toggle to save memory)
- Simple sphere-based self-collision check
- Smooth trajectory planner (linear interpolation in joint space)
- Benchmarking utility for IK
- Keyboard interactive controls for manual joint nudging

Designed to be lightweight and runnable on modest hardware (Ryzen 5 4600H, 8GB RAM, Vega iGPU).

Dependencies:
- Python 3.8+
- numpy
- matplotlib

Run:
    python 4DOF_Robotic_Arm_Simulation.py

Author: ChatGPT (assistant)
"""

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation
import time


# ---------------------- fast rotation helpers ----------------------
# these are already quite fast using math.cos/sin; kept for clarity and
# to allow swapping in approximations if needed for extreme performance.

def rot_z(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]], dtype=float)


def rot_y(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[ c, 0, s, 0],
                     [ 0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [ 0, 0, 0, 1]], dtype=float)


def trans_x(a):
    return np.array([[1, 0, 0, a],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)


class RoboticArm4DOF:
    """A simple, efficient 4-DOF revolute arm with utilities.

    Joint layout (convention used here):
       joint0: base yaw (rotation about Z)
       joint1: shoulder pitch (rotation about Y)
       joint2: elbow pitch (rotation about Y)
       joint3: wrist pitch (rotation about Y)

    Links are along the local x-axis after rotations, using simple transforms.
    """

    def __init__(self, link_lengths=None, angle_limits=None):
        if link_lengths is None:
            link_lengths = [0.6, 0.5, 0.4, 0.25]
        self.L = np.array(link_lengths, dtype=float)
        assert self.L.size == 4

        if angle_limits is None:
            angle_limits = [(-math.pi, math.pi),
                            (-math.pi/2, math.pi/2),
                            (-math.pi/2, math.pi/2),
                            (-math.pi/2, math.pi/2)]
        self.limits = angle_limits

    def forward_kinematics(self, thetas):
        t0, t1, t2, t3 = thetas
        T = np.eye(4)
        positions = [T[:3, 3].copy()]

        T = T @ rot_z(t0)
        T = T @ trans_x(self.L[0])
        positions.append(T[:3, 3].copy())

        T = T @ rot_y(t1)
        T = T @ trans_x(self.L[1])
        positions.append(T[:3, 3].copy())

        T = T @ rot_y(t2)
        T = T @ trans_x(self.L[2])
        positions.append(T[:3, 3].copy())

        T = T @ rot_y(t3)
        T = T @ trans_x(self.L[3])
        positions.append(T[:3, 3].copy())

        return np.vstack(positions)

    def clamp_angles(self, thetas):
        out = np.empty_like(thetas)
        for i, th in enumerate(thetas):
            lo, hi = self.limits[i]
            out[i] = max(lo, min(hi, th))
        return out

    def reach(self):
        return float(np.sum(self.L))

    # ---------------------- robust joint axis in world coords ----------------------
    def get_joint_axis_world(self, thetas, joint_index):
        """Return the joint's rotation axis in world coordinates as a 3-vector.

        joint_index: 0..3
        """
        T = np.eye(4)
        # base rotation
        T = T @ rot_z(thetas[0])
        T = T @ trans_x(self.L[0])
        if joint_index == 0:
            return np.array([0.0, 0.0, 1.0])  # base yaw always Z

        # for pitch joints we need to step further
        if joint_index >= 1:
            T = T @ rot_y(thetas[1])
            T = T @ trans_x(self.L[1])
        if joint_index >= 2:
            T = T @ rot_y(thetas[2])
            T = T @ trans_x(self.L[2])
        # joint local y-axis in world coords
        axis_world = (T @ np.array([0.0, 1.0, 0.0, 0.0]))[:3]
        # normalize
        n = np.linalg.norm(axis_world)
        if n < 1e-8:
            return np.array([0.0, 1.0, 0.0])
        return axis_world / n

    # ---------------------- CCD IK (uses get_joint_axis_world) ----------------------
    def ik_ccd(self, target, thetas_init=None, max_iters=200, tol=1e-3, damping=1.0, record_history=True):
        if thetas_init is None:
            thetas = np.zeros(4, dtype=float)
        else:
            thetas = np.array(thetas_init, dtype=float)
        target = np.array(target, dtype=float)
        history = [] if record_history else None

        if np.linalg.norm(target) > self.reach() + 1e-6:
            return thetas, False, history

        for it in range(max_iters):
            positions = self.forward_kinematics(thetas)
            if record_history:
                history.append(positions.copy())
            ee = positions[-1]
            err = np.linalg.norm(target - ee)
            if err <= tol:
                return self.clamp_angles(thetas), True, history

            for j in reversed(range(4)):
                positions = self.forward_kinematics(thetas)
                joint_pos = positions[j]
                ee = positions[-1]

                v1 = ee - joint_pos
                v2 = target - joint_pos
                n1 = np.linalg.norm(v1)
                n2 = np.linalg.norm(v2)
                if n1 < 1e-8 or n2 < 1e-8:
                    continue

                # project both vectors into plane orthogonal to joint axis
                axis = self.get_joint_axis_world(thetas, j)
                proj_v1 = v1 - np.dot(v1, axis) * axis
                proj_v2 = v2 - np.dot(v2, axis) * axis
                nproj1 = np.linalg.norm(proj_v1)
                nproj2 = np.linalg.norm(proj_v2)
                if nproj1 < 1e-8 or nproj2 < 1e-8:
                    continue
                p1u = proj_v1 / nproj1
                p2u = proj_v2 / nproj2
                cos_p = np.clip(np.dot(p1u, p2u), -1.0, 1.0)
                ang = math.acos(cos_p)
                sign = np.sign(np.dot(np.cross(p1u, p2u), axis))
                signed_ang = sign * ang

                # apply correction
                thetas[j] += signed_ang * damping
                lo, hi = self.limits[j]
                thetas[j] = max(lo, min(hi, thetas[j]))

            # end joint loop
        # end iterations
        positions = self.forward_kinematics(thetas)
        if record_history:
            history.append(positions.copy())
        ee = positions[-1]
        if np.linalg.norm(target - ee) <= tol:
            return self.clamp_angles(thetas), True, history
        return self.clamp_angles(thetas), False, history

    # ---------------------- simple self-collision check ----------------------
    def check_self_collision(self, positions, sphere_radius=0.08):
        # compare non-adjacent link origins for proximity
        n = positions.shape[0]
        for i in range(n - 1):
            for j in range(i + 2, n - 1):  # skip adjacent links
                if np.linalg.norm(positions[i] - positions[j]) < sphere_radius:
                    return True
        return False

    # ---------------------- smooth joint-space trajectory ----------------------
    def move_to_target_smooth(self, start_angles, target, steps=50, ik_max_iters=200, tol=1e-3):
        thetas_final, success, history = self.ik_ccd(target, start_angles, max_iters=ik_max_iters, tol=tol, damping=0.9, record_history=False)
        if not success:
            return None, False
        trajectories = []
        for step in range(steps):
            alpha = step / float(max(1, steps - 1))
            current_angles = start_angles + alpha * (thetas_final - start_angles)
            poses = self.forward_kinematics(current_angles)
            trajectories.append(poses)
        return trajectories, True

    # ---------------------- benchmarking utility ----------------------
    def benchmark_ik(self, num_targets=100, seed=42, max_iters=200):
        rng = np.random.default_rng(seed)
        times = []
        successes = 0
        for i in range(num_targets):
            # sample a target within reach sphere
            r = rng.uniform(low=0.1, high=self.reach() * 0.95)
            theta = rng.uniform(-math.pi, math.pi)
            phi = rng.uniform(-math.pi/4, math.pi/3)
            x = r * math.cos(phi) * math.cos(theta)
            y = r * math.cos(phi) * math.sin(theta)
            z = abs(r * math.sin(phi))  # prefer non-negative z
            target = np.array([x, y, z])

            t0 = time.time()
            _, success, _ = self.ik_ccd(target, max_iters=max_iters, record_history=False)
            dt = (time.time() - t0)
            times.append(dt)
            successes += 1 if success else 0

        times = np.array(times)
        return {
            'avg_ms': float(times.mean() * 1000),
            'median_ms': float(np.median(times) * 1000),
            'std_ms': float(times.std() * 1000),
            'success_rate': successes / float(num_targets)
        }


# ---------------------- plotting helpers & animation ----------------------

def plot_arm(ax, positions, label=None, linewidth=4):
    xs = positions[:, 0]
    ys = positions[:, 1]
    zs = positions[:, 2]
    ax.plot(xs, ys, zs, marker='o', linewidth=linewidth)
    if label:
        ax.text(xs[-1], ys[-1], zs[-1], label)


def animate_reaching(arm, target, thetas_initial=None, dpi=80, interval=80, show_axes=True, record_history=True, interactive=False):
    thetas0 = thetas_initial if thetas_initial is not None else np.zeros(4)
    thetas, success, history = arm.ik_ccd(target, thetas0, max_iters=300, tol=1e-3, damping=0.9, record_history=record_history)

    if history is None or len(history) == 0:
        history = [arm.forward_kinematics(thetas0), arm.forward_kinematics(thetas)]

    fig = plt.figure(figsize=(6, 6), dpi=dpi)
    ax = fig.add_subplot(111, projection='3d')
    r = arm.reach() * 1.1
    ax.set_xlim(-r, r)
    ax.set_ylim(-r, r)
    ax.set_zlim(0, r * 1.1)
    if not show_axes:
        ax.set_axis_off()

    ax.scatter([target[0]], [target[1]], [target[2]], c='red', s=50)
    line, = ax.plot([], [], [], marker='o', linewidth=4)
    title = ax.set_title('4DOF Robotic Arm - CCD IK')

    def update(frame):
        ax.collections.clear()
        ax.artists.clear()
        poses = history[min(frame, len(history) - 1)]
        xs = poses[:, 0]
        ys = poses[:, 1]
        zs = poses[:, 2]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)
        ax.scatter([target[0]], [target[1]], [target[2]], c='red', s=50)
        title.set_text(f'Frame {frame+1}/{len(history)}')
        return line,

    ani = FuncAnimation(fig, update, frames=len(history) + 10, interval=interval, blit=False)

    if success:
        print("IK converged: target reached.")
    else:
        final_ee = arm.forward_kinematics(thetas)[-1]
        print("IK did NOT converge fully. Final distance:", np.linalg.norm(final_ee - target))

    # Interactive nudging via keyboard if requested
    if interactive:
        current_angles = thetas.copy()

        def on_key(event):
            nonlocal current_angles
            step = 0.05
            if event.key == 'q':
                current_angles[0] += step
            elif event.key == 'a':
                current_angles[0] -= step
            elif event.key == 'w':
                current_angles[1] += step
            elif event.key == 's':
                current_angles[1] -= step
            elif event.key == 'e':
                current_angles[2] += step
            elif event.key == 'd':
                current_angles[2] -= step
            elif event.key == 'r':
                current_angles[3] += step
            elif event.key == 'f':
                current_angles[3] -= step
            elif event.key == 'space':
                # try to solve to the last target from new pose
                new_thetas, new_success, new_hist = arm.ik_ccd(target, current_angles, max_iters=200, record_history=True)
                if new_hist is not None:
                    history[:] = new_hist
                print('Re-solved from nudged pose, success=', new_success)
                return
            current_angles = arm.clamp_angles(current_angles)
            # update last frame shown to reflect new angles
            poses = arm.forward_kinematics(current_angles)
            line.set_data(poses[:, 0], poses[:, 1])
            line.set_3d_properties(poses[:, 2])
            fig.canvas.draw_idle()

        fig.canvas.mpl_connect('key_press_event', on_key)
        print("Interactive mode: use q/a, w/s, e/d, r/f to nudge joints 0..3, press space to re-solve")

    plt.show()
    return ani, success


# ---------------------- small demo & CLI-style utilities ----------------------

def demo_random_targets(num=3, interactive=False):
    arm = RoboticArm4DOF()
    thetas0 = np.array([0.0, -0.4, 0.8, -0.2])
    rng = np.random.default_rng(123)
    for i in range(num):
        r = rng.uniform(low=0.2, high=arm.reach() * 0.95)
        theta = rng.uniform(-math.pi / 2, math.pi / 2)
        phi = rng.uniform(0, math.pi / 2)
        x = r * math.cos(phi) * math.cos(theta)
        y = r * math.cos(phi) * math.sin(theta)
        z = r * math.sin(phi)
        target = np.array([x, y, z])
        print(f"Demo {i+1}: target = {target}")
        animate_reaching(arm, target, thetas_initial=thetas0, interval=60, record_history=True, interactive=interactive)


def run_benchmark():
    arm = RoboticArm4DOF()
    stats = arm.benchmark_ik(num_targets=200)
    print("IK benchmark (200 targets):")
    print(f" avg: {stats['avg_ms']:.2f} ms, median: {stats['median_ms']:.2f} ms, std: {stats['std_ms']:.2f} ms")
    print(f" success rate: {stats['success_rate']*100:.1f}%")


if __name__ == '__main__':
    print("4DOF Robotic Arm Simulation (enhanced)")
    print("Default link lengths: ", [0.6, 0.5, 0.4, 0.25])
    print("Options: run demo_random_targets(), run_benchmark(), or call animate_reaching()")
    # run small demo by default
    demo_random_targets(2)
