"""WCR controller for Webots (supervisor): EDF adhesion + 4WIS drive + keyboard / tests.

Models (adhesion, 4WIS kinematics) come from wcr_webots/wcr_webots/model.py, shared with the
ROS 2 driver plugin (wcr_webots.wcr_driver, used when the robot runs with controller <extern>).

controllerArgs:
  keyboard                         arrows: drive / turn, A/D: crab sideways, space: stop
  climb <distance m> <speed m/s> [quit] [snapshot]
                                   scripted settle -> climb -> hold, PASS/FAIL summary + CSV in results/
  drive [quit]                     up / turn / crab / down sequence, reports blow-ups
  sensors [quit]                   climb 1 m, save wall + front camera images, lidar scan, sensor summary

Sensors (Phase 2): front_camera, wall_camera (+ inspection SpotLight), lidar (2D, parallel to the
wall), imu, gyro, accelerometer, <wheel>_wheel_sensor / <wheel>_steering_sensor encoders.
"""
import csv
import math
import os
import sys

from controller import Keyboard, Supervisor

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'wcr_webots'))
from wcr_webots.model import WHEELS, WHEEL_R, Adhesion, edf_speed, inverse_kinematics  # noqa: E402  (shared with ROS)

RESULTS = os.path.join(REPO, 'results')


class Wcr:
    def __init__(self):
        self.robot = Supervisor()
        self.dt = int(self.robot.getBasicTimeStep())
        self.node = self.robot.getSelf()
        self.wheel = {n: self.robot.getDevice(f'{n}_wheel') for n in WHEELS}
        self.steer = {n: self.robot.getDevice(f'{n}_steering') for n in WHEELS}
        for m in self.wheel.values():
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
        self.imu = self.robot.getDevice('imu')
        self.imu.enable(self.dt)
        for name in ('gyro', 'accelerometer'):
            self.robot.getDevice(name).enable(self.dt)
        self.enc = {n: self.robot.getDevice(f'{n}_wheel_sensor') for n in WHEELS}
        for s in self.enc.values():
            s.enable(self.dt)
        self.front_camera = self.robot.getDevice('front_camera')
        self.front_camera.enable(64)                       # ~15 Hz
        self.wall_camera = self.robot.getDevice('wall_camera')
        self.wall_camera.enable(64)
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(96)                              # ~10 Hz (multiple of the 8 ms step)
        self.adhesion = Adhesion()
        self.steer_cmd = {n: 0.0 for n in WHEELS}          # last steering commands (kinematics hysteresis)
        self.edf = self.robot.getDevice('edf_motor')       # visual EDF rotor
        self.edf.setPosition(float('inf'))
        self.edf_sensor = self.robot.getDevice('edf_sensor')
        self.edf_sensor.enable(self.dt)

    @property
    def force(self):
        return self.adhesion.force

    @property
    def beta(self):
        return self.adhesion.beta

    def step(self, vx=0.0, vy=0.0, wz=0.0):
        for n, (om, d) in inverse_kinematics(vx, vy, wz, self.steer_cmd).items():
            self.steer[n].setPosition(d)
            self.wheel[n].setVelocity(om)
            self.steer_cmd[n] = d
        f = self.adhesion.update(self.imu.getQuaternion(), self.dt / 1000.0)   # x y z w, world ENU
        self.node.addForce([0.0, 0.0, -f], True)           # into the surface, robot frame
        self.edf.setVelocity(edf_speed(f))
        return self.robot.step(self.dt) != -1

    def pos(self):
        return self.node.getPosition()


def keyboard(w):
    kb = w.robot.getKeyboard()
    kb.enable(w.dt)
    print('WCR keyboard: UP/DOWN drive up/down the face, LEFT/RIGHT turn, A/D crab, SPACE stop. '
          'Click the 3D view first.', flush=True)
    vx = vy = wz = 0.0
    t_print = 0.0
    while True:
        k = kb.getKey()
        while k != -1:
            if k == Keyboard.UP: vx, vy, wz = 0.12, 0.0, 0.0
            elif k == Keyboard.DOWN: vx, vy, wz = -0.12, 0.0, 0.0
            elif k == Keyboard.LEFT: vx, vy, wz = 0.0, 0.0, 0.5
            elif k == Keyboard.RIGHT: vx, vy, wz = 0.0, 0.0, -0.5
            elif k == ord('A'): vx, vy, wz = 0.0, 0.12, 0.0
            elif k == ord('D'): vx, vy, wz = 0.0, -0.12, 0.0
            elif k == ord(' '): vx = vy = wz = 0.0
            k = kb.getKey()
        if not w.step(vx, vy, wz):
            return
        if w.robot.getTime() - t_print > 2.0 and w.beta is not None:
            t_print = w.robot.getTime()
            p = w.pos()
            print(f't {t_print:6.1f} s  z {p[2]:6.2f} m  surface {math.degrees(w.beta):5.1f} deg  '
                  f'adhesion {w.force:5.1f} N', flush=True)


def climb_test(w, distance, speed, quit_after, snapshot):
    os.makedirs(RESULTS, exist_ok=True)
    rows, phase, t_phase = [], 'settle', 0.0
    start = last = w.pos()
    travel, res = 0.0, {}
    while True:
        t = w.robot.getTime()
        v = speed if phase == 'climb' else 0.0
        if not w.step(v):
            return 1
        p = w.pos()
        travel += math.dist(p, last)
        last = p
        rows.append((round(t, 3), phase, *[round(a, 4) for a in p],
                     round(math.degrees(w.beta or 0), 2), round(w.force, 2)))
        el = t - t_phase
        if phase == 'settle' and el >= 3.0:
            res['settle_slide_m'] = math.dist(p, start)
            phase, t_phase, travel = 'climb', t, 0.0
        elif phase == 'climb' and (travel >= distance or el > 3 * distance / speed):
            res['climb_speed'] = travel / el
            res['climb_height_m'] = p[2] - start[2]
            res['surface_deg'] = math.degrees(w.beta)
            phase, t_phase, p_hold = 'hold', t, p
        elif phase == 'hold':
            if el < 0.5:
                p_hold = p
            elif el >= 3.0:
                res['hold_drift_m'] = math.dist(p, p_hold)
                break
        if p[2] < start[2] - 0.05:
            res['fell'] = 1.0
            break
    ok = (not res.get('fell') and res.get('climb_speed', 0) >= 0.8 * speed and res.get('hold_drift_m', 1) <= 0.005)
    with open(os.path.join(RESULTS, 'webots_climb.csv'), 'w', newline='') as f:
        csv.writer(f).writerows([('t', 'phase', 'x', 'y', 'z', 'surface_deg', 'adhesion_N')] + rows)
    lines = ['WEBOTS climb test: ' + ('PASS' if ok else 'FAIL')] + [f'  {k:15s} {v:.4f}' for k, v in res.items()]
    lines.append(f'  commanded speed {speed:.4f}   sim time {w.robot.getTime():.1f} s')
    print('\n'.join(lines), flush=True)
    open(os.path.join(RESULTS, 'webots_climb.txt'), 'w').write('\n'.join(lines) + '\n')
    if snapshot:
        w.robot.exportImage(os.path.join(RESULTS, 'webots_climb.png'), 90)
        w.robot.step(w.dt)
    if quit_after:
        w.robot.simulationQuit(0 if ok else 1)
    while w.step():
        pass
    return 0


def drive_test(w, quit_after):
    """Keyboard-like sequence (up, turn, crab, turn back, down) on the face; reports blow-ups."""
    seq = [('settle', 0, 0, 0, 3), ('up', 0.12, 0, 0, 8), ('turn_left', 0, 0, 0.5, 4), ('crab_left', 0, 0.12, 0, 4),
           ('crab_right', 0, -0.12, 0, 4), ('turn_right', 0, 0, -0.5, 4), ('down', -0.12, 0, 0, 6), ('stop', 0, 0, 0, 3)]
    os.makedirs(RESULTS, exist_ok=True)
    report, ok = [], True
    for name, vx, vy, wz, dur in seq:
        t0, p0 = w.robot.getTime(), w.pos()
        while w.robot.getTime() - t0 < dur:
            if not w.step(vx, vy, wz):
                return
            p, v = w.pos(), w.node.getVelocity()
            if not all(math.isfinite(a) and abs(a) < 1e3 for a in list(p) + list(v)):
                ok = False
                report.append(f'{name:10s} BLEW UP at t={w.robot.getTime():.3f}s pos={p} vel={v}')
                break
        if not ok:
            break
        p = w.pos()
        report.append(f'{name:10s} moved {math.dist(p, p0):5.3f} m  z {p[2]:6.3f}  surface '
                      f'{math.degrees(w.beta or 0):5.1f} deg  adhesion {w.force:4.1f} N  '
                      f'off-plane speed {abs(w.node.getVelocity()[2]):.3f}')
    lines = ['WEBOTS drive test: ' + ('PASS' if ok else 'FAIL')] + report
    print('\n'.join(lines), flush=True)
    open(os.path.join(RESULTS, 'webots_drive_test.txt'), 'w').write('\n'.join(lines) + '\n')
    if quit_after:
        w.robot.simulationQuit(0 if ok else 1)


def sensor_test(w, quit_after):
    """Settle, climb 1 m, stop; save both camera images, the lidar scan (CSV) and a summary of
    every sensor to results/webots_sensors.*"""
    os.makedirs(RESULTS, exist_ok=True)
    enc0 = None
    for phase, v, dur in (('settle', 0.0, 3.0), ('climb', 0.12, 1.0 / 0.12), ('stop', 0.0, 1.0)):
        t0 = w.robot.getTime()
        while w.robot.getTime() - t0 < dur:
            if not w.step(v):
                return
            if enc0 is None and phase == 'climb':
                enc0 = {n: s.getValue() for n, s in w.enc.items()}
    w.wall_camera.saveImage(os.path.join(RESULTS, 'webots_sensors_wall_camera.png'), 95)
    w.front_camera.saveImage(os.path.join(RESULTS, 'webots_sensors_front_camera.png'), 95)
    ranges = w.lidar.getRangeImage()
    fov = w.lidar.getFov()
    with open(os.path.join(RESULTS, 'webots_sensors_lidar.csv'), 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(('angle_rad', 'range_m'))
        for i, r in enumerate(ranges):
            wr.writerow((round(fov / 2 - i * fov / len(ranges), 5), r if math.isfinite(r) else 'inf'))
    hits = [r for r in ranges if math.isfinite(r)]
    enc = {n: s.getValue() - enc0[n] for n, s in w.enc.items()}
    g = w.robot.getDevice('gyro').getValues()
    a = w.robot.getDevice('accelerometer').getValues()
    rpy = w.imu.getRollPitchYaw()
    lines = ['WEBOTS sensor test: ' + ('PASS' if len(ranges) == 360 and all(abs(v) > 0 for v in enc.values()) else 'FAIL'),
             f'  lidar         {len(ranges)} beams, {len(hits)} hits'
             + (f', nearest {min(hits):.2f} m, farthest {max(hits):.2f} m' if hits else ''),
             f'  wall camera   {w.wall_camera.getWidth()}x{w.wall_camera.getHeight()}, fov {w.wall_camera.getFov():.2f} rad',
             f'  front camera  {w.front_camera.getWidth()}x{w.front_camera.getHeight()}, fov {w.front_camera.getFov():.2f} rad',
             f'  imu rpy       {", ".join(f"{math.degrees(x):6.1f}" for x in rpy)} deg   surface {math.degrees(w.beta):.1f} deg',
             f'  gyro          {", ".join(f"{x:+.3f}" for x in g)} rad/s',
             f'  accelerometer {", ".join(f"{x:+.2f}" for x in a)} m/s2',
             '  encoders      ' + ', '.join(f'{n} {d * WHEEL_R:+.3f} m' for n, d in enc.items()) + '  (1 m climbed)',
             f'  EDF rotor     {w.edf_sensor.getValue() / (2 * math.pi):.0f} turns in {w.robot.getTime():.1f} s']
    print('\n'.join(lines), flush=True)
    open(os.path.join(RESULTS, 'webots_sensors.txt'), 'w').write('\n'.join(lines) + '\n')
    w.robot.exportImage(os.path.join(RESULTS, 'webots_climb.png'), 90)
    w.robot.step(w.dt)
    open(os.path.join(RESULTS, 'webots_climb.txt'), 'w').write('\n'.join(lines) + '\n')   # for win_webots_test
    if quit_after:
        w.robot.simulationQuit(0)


def main():
    args = sys.argv[1:] or ['keyboard']
    w = Wcr()
    if args[0] == 'sensors':
        sensor_test(w, 'quit' in args)
    elif args[0] == 'drive':
        drive_test(w, 'quit' in args)
    elif args[0] == 'climb':
        dist = float(args[1]) if len(args) > 1 else 3.0
        speed = float(args[2]) if len(args) > 2 else 0.12
        climb_test(w, dist, speed, 'quit' in args, 'snapshot' in args)
    else:
        keyboard(w)


main()
