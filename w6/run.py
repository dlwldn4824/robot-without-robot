"""W6 실행기 — MuJoCo 시뮬레이션 루프.

  Sensor 읽기 -> controller.decide_action() -> 바퀴 속도 -> mj_step -> 반복
  (= Closed-loop)

사용 예:
  python run.py --duration 40 --log runs/basic.json
  python run.py --controller improved --noise 0.15 --threshold 0.7
"""
import argparse, json, math, os, random
import numpy as np
import mujoco

MAX_WHEEL_SPEED = 10.0      # [rad/s] 정규화 1.0 에 해당하는 바퀴 각속도
CONTROL_HZ      = 20        # 컨트롤러 호출 주기
RANGE_MAX       = 5.0       # 센서 최대 거리 (아무것도 안 맞으면 이 값으로 본다)

ROBOT_GEOMS = {'body', 'nose', 'tire_left', 'tire_right', 'caster_f', 'caster_b'}
HERE = os.path.dirname(os.path.abspath(__file__))


def yaw_of(qpos):
    w, x, y, z = qpos[3:7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def read_sensors(model, data, noise=0.0, rng=None):
    """거리 3개를 읽는다. MuJoCo rangefinder 는 아무것도 안 맞으면 -1 을 준다."""
    out = []
    for name in ('left', 'front', 'right'):
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        v = float(data.sensordata[model.sensor_adr[sid]])
        if v < 0:                      # -1 = 측정 실패(아무것도 없음)
            v = RANGE_MAX
        if noise > 0:                  # 과제 6-C: 센서 노이즈
            v = max(0.0, v + rng.uniform(-noise, noise))
        out.append(v)
    return out                          # [left, front, right]


def count_collisions(model, data):
    """로봇 geom 이 벽/장애물에 닿은 접촉 수 (바닥 제외)"""
    n = 0
    for i in range(data.ncon):
        c = data.contact[i]
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
        names = {g1, g2}
        if names & ROBOT_GEOMS and not (names & {'floor'}):
            if names - ROBOT_GEOMS:     # 상대가 벽이나 장애물
                n += 1
    return n


def simulate(args):
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'world.xml'))
    data = mujoco.MjData(model)
    rng = random.Random(args.seed)

    if args.controller == 'improved':
        from controller_improved import ImprovedController
        obj = ImprovedController(enter_th=args.threshold, exit_th=args.threshold + 0.3,
                                 hold_s=0.45, avg_n=5)
        decide = lambda l, f, r, t: obj.decide(l, f, r, t)
    else:
        import controller
        decide = lambda l, f, r, t: controller.decide_action(l, f, r, args.threshold)
    import controller as C

    data.qpos[0:2] = args.start
    mujoco.mj_forward(model, data)

    steps_per_ctrl = max(1, int(round((1.0 / CONTROL_HZ) / model.opt.timestep)))
    n_steps = int(args.duration / model.opt.timestep)

    frames, log = [], []
    renderer = None
    if args.gif:
        renderer = mujoco.Renderer(model, 480, 640)
        cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
        cam.azimuth, cam.elevation, cam.distance = 90, -89, 9.6
        cam.lookat[:] = [0, 0, 0]

    action = 'STOP'
    collisions, prev_xy, path_len = 0, data.qpos[:2].copy(), 0.0
    turn_steps, in_collision = 0, False
    gif_every = max(1, int(round((1.0 / args.fps) / model.opt.timestep)))

    for k in range(n_steps):
        if k % steps_per_ctrl == 0:
            L, F, R = read_sensors(model, data, args.noise, rng)
            action = decide(L, F, R, data.time)
            wl, wr = C.action_to_wheels(action)
            data.ctrl[0] = wl * MAX_WHEEL_SPEED * args.speed
            data.ctrl[1] = wr * MAX_WHEEL_SPEED * args.speed
            log.append(dict(t=round(data.time, 2), L=round(L, 3), F=round(F, 3), R=round(R, 3),
                            action=action, state=C.check_state(F),
                            x=round(float(data.qpos[0]), 3), y=round(float(data.qpos[1]), 3),
                            yaw=round(math.degrees(yaw_of(data.qpos)), 1)))
            if action.startswith('TURN'):
                turn_steps += 1

        mujoco.mj_step(model, data)

        hit = count_collisions(model, data) > 0
        if hit and not in_collision:
            collisions += 1
        in_collision = hit

        xy = data.qpos[:2].copy()
        path_len += float(np.linalg.norm(xy - prev_xy))
        prev_xy = xy

        if renderer is not None and k % gif_every == 0:
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render())

    n_ctrl = len(log)
    result = dict(
        controller=args.controller, threshold=args.threshold, noise=args.noise,
        speed=args.speed, duration=args.duration,
        collisions=collisions, path_len=round(path_len, 2),
        turn_ratio=round(turn_steps / max(n_ctrl, 1), 3),
        action_changes=sum(1 for a, b in zip(log, log[1:]) if a['action'] != b['action']),
        min_front=round(min(e['F'] for e in log), 3),
        final_xy=[round(float(data.qpos[0]), 2), round(float(data.qpos[1]), 2)],
        ctrl_steps=n_ctrl,
    )

    if args.gif:
        import imageio
        os.makedirs(os.path.dirname(args.gif) or '.', exist_ok=True)
        imageio.mimsave(args.gif, frames, duration=1.0 / args.fps, loop=0)
        result['gif'] = args.gif
    if args.log:
        os.makedirs(os.path.dirname(args.log) or '.', exist_ok=True)
        with open(args.log, 'w') as f:
            json.dump(dict(result=result, log=log), f, ensure_ascii=False)
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--controller', default='basic', choices=['basic', 'improved'])
    p.add_argument('--threshold', type=float, default=0.7)
    p.add_argument('--noise', type=float, default=0.0)
    p.add_argument('--speed', type=float, default=1.0)
    p.add_argument('--duration', type=float, default=40.0)
    p.add_argument('--start', type=float, nargs=2, default=[0.0, 0.0])
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--gif', default=None)
    p.add_argument('--fps', type=float, default=25)
    p.add_argument('--log', default=None)
    args = p.parse_args()
    print(json.dumps(simulate(args), ensure_ascii=False))


if __name__ == '__main__':
    main()
