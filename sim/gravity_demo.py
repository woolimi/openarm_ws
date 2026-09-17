"""중력보상 계수를 바꾼 팔 셋을 나란히 돌린다.

시뮬레이션에서는 보상에 쓰는 모델과 움직이는 물체가 같은 URDF 에서 나오므로 k=1 이
정의상 정확한 보상이다. 실측 캘리브레이션 값은 쓰지 않는다 — 실물에서 k 가 1 을
벗어나는 원인(미지 페이로드)은 키 P 로 따로 보여 준다.

    k < 1   중력을 다 못 이겨 팔이 처진다
    k = 1   어느 자세에서든 제자리에 뜬다. 손으로 밀면 민 자리에 그대로 선다
    k > 1   중력보다 센 토크가 남아 팔이 위로 떠오른다

조작
    F   세 팔 손끝에 같은 크기의 외력 펄스
    P   손 링크에 페이로드를 얹는다(플랜트만 안다)
    R   시작 자세로 되돌린다
    마우스 ctrl + 오른쪽 드래그로 한 팔만 직접 밀 수도 있다
"""

import time

import mujoco
import mujoco.viewer
import numpy as np

import config
import model as model_builder

ARM_JOINTS = config.ARM_JOINTS
FINGER_JOINTS = config.FINGER_JOINTS


class Arm:
    """한 팔의 인덱스 묶음과 보상 계수."""

    def __init__(self, plant, name, gain, label):
        self.name = name
        self.gain = gain
        self.label = label
        self.joint_ids = np.array(
            [plant.joint(f'{name}_{j}').id for j in ARM_JOINTS])
        self.dofs = np.array(
            [plant.joint(f'{name}_{j}').dofadr[0] for j in ARM_JOINTS])
        self.qpos = np.array(
            [plant.joint(f'{name}_{j}').qposadr[0] for j in ARM_JOINTS])
        self.finger_dofs = np.array(
            [plant.joint(f'{name}_{j}').dofadr[0] for j in FINGER_JOINTS])
        self.tip = plant.body(f'{name}_{config.TIP_BODY}').id
        self.payload_body = plant.body(f'{name}_{config.PAYLOAD_BODY}').id
        self.nominal_mass = float(plant.body_mass[self.payload_body])


class Demo:
    def __init__(self):
        urdf = model_builder.generate_urdf()
        #: 움직이는 쪽. 페이로드를 얹으면 이 모델의 질량만 바뀐다.
        self.plant = model_builder.build_scene(urdf)
        #: 보상토크를 계산하는 쪽. URDF 그대로이고 페이로드를 모른다.
        self.nominal = model_builder.build_scene(urdf)

        self.data = mujoco.MjData(self.plant)
        self.nominal_data = mujoco.MjData(self.nominal)
        self.arms = [Arm(self.plant, name, gain, label)
                     for name, gain, label, _rgba in config.ARMS]

        self.push_until = -1.0
        self.push_direction = np.array(config.PUSH_DIRECTION, dtype=float)
        self.payload_on = False
        self.comp_on = config.COMP_ENABLED_AT_START
        self.hold_until = 0.0
        self.torque = np.zeros(self.plant.nv)
        self.rng = np.random.default_rng()
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.plant, self.data)
        for arm in self.arms:
            self.data.qpos[arm.qpos] = config.START_QPOS
        self.data.qvel[:] = 0.0
        self.push_until = -1.0
        self.hold_until = time.perf_counter() + config.RESET_HOLD
        mujoco.mj_forward(self.plant, self.data)

    def gravity_torque(self):
        """지금 자세에서 URDF 모델이 말하는 중력토크.

        속도를 0 으로 둔 자세에서 bias 항은 코리올리 없이 중력만 남는다.
        """
        self.nominal_data.qpos[:] = self.data.qpos
        self.nominal_data.qvel[:] = 0.0
        self.nominal_data.qacc[:] = 0.0
        mujoco.mj_forward(self.nominal, self.nominal_data)
        return self.nominal_data.qfrc_bias

    def apply_compensation(self):
        """보상 토크를 그대로 관절에 싣는다.

        목표 자세도, 위치 오차도 쓰지 않는다 — 강성과 감쇠 gain 은 0 이고 앞먹임
        토크 k·G(q) 하나뿐이다. 실물 모터의 MIT 모드로 치면 kp=kd=0 에 τ_ff 만
        보내는 것과 같다.
        """
        gravity = self.gravity_torque()
        self.torque[:] = 0.0
        for arm in self.arms:
            if self.comp_on:
                self.torque[arm.dofs] = arm.gain * gravity[arm.dofs]
            if config.LOCK_FINGERS:
                self.data.qvel[arm.finger_dofs] = 0.0
                self.torque[arm.finger_dofs] = gravity[arm.finger_dofs]
        self.data.qfrc_applied[:] = self.torque

    def pick_push_direction(self):
        """이번 펄스의 방향. 세 팔이 같은 방향으로 밀리도록 한 번만 뽑는다."""
        if not config.PUSH_RANDOM_DIRECTION:
            direction = np.array(config.PUSH_DIRECTION, dtype=float)
            return direction / np.linalg.norm(direction)

        direction = self.rng.normal(size=3)
        direction /= np.linalg.norm(direction)
        floor = config.PUSH_MIN_VERTICAL
        if floor > 0.0 and abs(direction[2]) < floor:
            sign = 1.0 if direction[2] >= 0.0 else -1.0
            horizontal = direction[:2]
            scale = np.sqrt(max(1.0 - floor ** 2, 0.0)) / max(
                np.linalg.norm(horizontal), 1e-9)
            direction = np.array([*(horizontal * scale), sign * floor])
        return direction

    def apply_push(self):
        active = self.data.time < self.push_until
        self.data.xfrc_applied[:] = 0.0
        if not active:
            return
        force = config.PUSH_FORCE * self.push_direction
        for arm in self.arms:
            self.data.xfrc_applied[arm.tip, :3] = force

    def toggle_payload(self):
        self.payload_on = not self.payload_on
        for arm in self.arms:
            extra = config.PAYLOAD_MASS if self.payload_on else 0.0
            self.plant.body_mass[arm.payload_body] = arm.nominal_mass + extra

    def on_key(self, keycode):
        key = chr(keycode).upper() if 0 < keycode < 0x110000 else ''
        if key == 'F':
            self.push_direction = self.pick_push_direction()
            self.push_until = self.data.time + config.PUSH_DURATION
            arrow = ' '.join(f'{v:+.2f}' for v in self.push_direction)
            print(f'외력 {config.PUSH_FORCE:g} N, {config.PUSH_DURATION:g} s, '
                  f'방향 [{arrow}] — 세 팔 손끝에 같이')
        elif key == 'P':
            self.toggle_payload()
            state = f'{config.PAYLOAD_MASS:g} kg 얹음' if self.payload_on else '내림'
            print(f'페이로드 {state} (보상 모델은 모름)')
        elif key == 'R':
            self.reset()
            print('시작 자세로 복귀')
        elif key == 'C':
            self.comp_on = not self.comp_on
            print(f"중력보상 {'ON' if self.comp_on else 'OFF'}")

    def draw_overlay(self, viewer, keep=0):
        """라벨과 화살표. 관절마다 실제로 실린 보상 토크를 축 방향 화살표로 그린다.

        `keep` 은 지우지 않고 남길 geom 개수다. 뷰어의 user_scn 은 우리 것만 담으므로
        0 이고, 오프스크린 렌더처럼 로봇이 이미 들어 있는 씬에 덧그릴 때만 쓴다.
        """
        scene = viewer.user_scn
        scene.ngeom = keep
        pushing = self.data.time < self.push_until
        for arm in self.arms:
            tip = self.data.xpos[arm.tip]
            torque = self.torque[arm.dofs]
            text = arm.label
            if self.comp_on:
                text += f'   torque {np.abs(torque).sum():.1f} Nm'
            else:
                text += '   COMP OFF'
            if self.payload_on:
                text += f'   +{config.PAYLOAD_MASS:g}kg'
            self._label(scene, tip + np.array([0.0, 0.0, 0.45]), text)

            for joint, value in zip(arm.joint_ids, torque):
                length = config.TORQUE_ARROW_SCALE * value
                if abs(length) < 0.01:
                    continue
                anchor = self.data.xanchor[joint]
                axis = self.data.xaxis[joint]
                self._arrow(scene, anchor, anchor + length * axis,
                            rgba=(1.0, 0.55, 0.1, 1.0), width=0.015)

            if pushing:
                # 화살촉이 손끝에 닿게 그린다 — 미는 힘으로 읽힌다.
                tail = tip - config.PUSH_ARROW_LENGTH * self.push_direction
                self._arrow(scene, tail, tip, rgba=(1.0, 0.25, 0.25, 1.0),
                            width=config.PUSH_ARROW_WIDTH)
                self._label(scene, tail, f'{config.PUSH_FORCE:g} N')

    @staticmethod
    def _label(scene, pos, text):
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom, mujoco.mjtGeom.mjGEOM_LABEL,
            np.array([0.2, 0.2, 0.2]), np.asarray(pos, dtype=float),
            np.eye(3).flatten(), np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32))
        geom.label = text
        scene.ngeom += 1

    @staticmethod
    def _arrow(scene, start, end, rgba=(1.0, 0.85, 0.2, 1.0), width=0.012):
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom, mujoco.mjtGeom.mjGEOM_ARROW,
            np.zeros(3), np.zeros(3), np.eye(3).flatten(),
            np.array(rgba, dtype=np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, width,
                             np.asarray(start, dtype=float),
                             np.asarray(end, dtype=float))
        scene.ngeom += 1

    def run(self):
        with mujoco.viewer.launch_passive(
                self.plant, self.data, key_callback=self.on_key) as viewer:
            viewer.cam.azimuth = config.CAMERA['azimuth']
            viewer.cam.elevation = config.CAMERA['elevation']
            viewer.cam.distance = config.CAMERA['distance']
            viewer.cam.lookat[:] = config.CAMERA['lookat']
            while viewer.is_running():
                start = time.perf_counter()
                self.apply_compensation()
                self.apply_push()
                if start >= self.hold_until:
                    mujoco.mj_step(self.plant, self.data)
                else:
                    self.data.qvel[:] = 0.0
                    mujoco.mj_forward(self.plant, self.data)
                self.draw_overlay(viewer)
                viewer.sync()
                period = config.TIMESTEP / config.REALTIME_FACTOR
                remaining = period - (time.perf_counter() - start)
                if remaining > 0:
                    time.sleep(remaining)


if __name__ == '__main__':
    Demo().run()
