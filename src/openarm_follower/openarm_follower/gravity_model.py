"""중력 모델과 회귀 행렬 — 캘리브레이션의 순수 계산 부분.

정지한 팔이 내는 관절 토크는 그 자세의 중력토크다. 모델이 아는 중력 G(q) 를 빼면
남는 잔차는 모델이 모르는 것뿐이고, 그 대부분은 손끝에 달린 미모델 질량이다. 점질량
하나의 기여는 θ = (m, m·cx, m·cy, m·cz) 에 대해 선형이라 자세 몇 개면 풀린다.

    τ_meas(q) = G(q) + A(q)·θ + b

    A[:, 0]     = g · Jᵥᵀ ẑ                  (질량)
    A[:, 1 + k] = g · J𝜔ᵀ ((R·e_k) × ẑ)      (무게중심, 팁 프레임)

b 는 관절별 상수 오프셋(모터 토크 영점 오차)이다. 같이 풀지 않으면 최소자승이 그
상수를 무게중심으로 잘못 설명해, 표본에 없는 자세에서 과보상이 난다.

모델은 하드웨어 플러그인이 쓰는 KDL 사슬과 같은 물체 집합이어야 한다. 플러그인은
root→tip 한 줄기만 보므로 여기서도 그 줄기 밖의 물체(손가락·반대쪽 팔)는 관성을
0 으로 지운다. 지운 손가락 질량은 잔차에 남아 payload 추정치가 대신 담는다.
"""

import numpy as np
import pinocchio

GRAVITY_ACCEL = 9.81

ARM_JOINT_COUNT = 7


def arm_joint_names(arm):
    return [f'openarm_{arm}_joint{i}' for i in range(1, ARM_JOINT_COUNT + 1)]


class GravityModel:
    """한 팔의 중력토크와 회귀 행렬. URDF XML 하나로 만들어진다."""

    def __init__(self, urdf_xml, arm, tip_link):
        self.arm = arm
        self.tip_link = tip_link
        self.model = pinocchio.buildModelFromXML(urdf_xml)

        self.joint_names = arm_joint_names(arm)
        missing = [n for n in self.joint_names if not self.model.existJointName(n)]
        if missing:
            raise ValueError(f'URDF 에 없는 관절: {missing}')
        if not self.model.existFrame(tip_link):
            raise ValueError(f'URDF 에 없는 팁 링크: {tip_link}')

        joint_ids = [self.model.getJointId(n) for n in self.joint_names]
        self.idx_q = [self.model.joints[i].idx_q for i in joint_ids]
        self.idx_v = [self.model.joints[i].idx_v for i in joint_ids]
        self.tip_frame = self.model.getFrameId(tip_link)

        self._strip_bodies_off_chain()
        self.data = self.model.createData()

        self.lower = np.array([self.model.lowerPositionLimit[i] for i in self.idx_q])
        self.upper = np.array([self.model.upperPositionLimit[i] for i in self.idx_q])

    def _strip_bodies_off_chain(self):
        """root→tip 줄기 밖의 관성을 지운다 (플러그인의 KDL 사슬과 같은 물체 집합)."""
        tip_joint = self.model.frames[self.tip_frame].parentJoint
        on_chain = {int(j) for j in self.model.supports[tip_joint]}
        for joint_id in range(1, self.model.njoints):
            if joint_id not in on_chain:
                self.model.inertias[joint_id] = pinocchio.Inertia.Zero()

    def configuration(self, joint_positions):
        """이 팔의 관절각을 넣은 전체 모델 자세. 나머지 관절은 0 이다 —
        어느 관절의 중력토크도 자기 아래 가지에만 달려 있어 영향이 없다."""
        q = pinocchio.neutral(self.model)
        for index, value in zip(self.idx_q, joint_positions):
            q[index] = float(value)
        return q

    def clamp(self, joint_positions):
        """URDF 관절 한계 안으로 자른다. 한계는 팔마다 다르다(좌우 오프셋)."""
        margin = 0.03
        return np.clip(np.asarray(joint_positions, dtype=float),
                       self.lower + margin, self.upper - margin)

    def nominal_gravity(self, joint_positions):
        """이 자세에서 팔을 들고 있는 데 필요한 관절토크 7개 [Nm]."""
        q = self.configuration(joint_positions)
        gravity = pinocchio.computeGeneralizedGravity(self.model, self.data, q)
        return np.array([gravity[i] for i in self.idx_v])

    def regression_rows(self, joint_positions):
        """이 자세의 회귀 행렬 A (7×4). 열은 (m, m·cx, m·cy, m·cz)."""
        q = self.configuration(joint_positions)
        pinocchio.computeJointJacobians(self.model, self.data, q)
        pinocchio.framesForwardKinematics(self.model, self.data, q)
        jacobian = pinocchio.getFrameJacobian(
            self.model, self.data, self.tip_frame,
            pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        linear = jacobian[:3, self.idx_v]
        angular = jacobian[3:, self.idx_v]
        rotation = self.data.oMf[self.tip_frame].rotation
        up = np.array([0.0, 0.0, 1.0])

        rows = np.zeros((ARM_JOINT_COUNT, 4))
        rows[:, 0] = GRAVITY_ACCEL * (linear.T @ up)
        for axis in range(3):
            rows[:, 1 + axis] = GRAVITY_ACCEL * (
                angular.T @ np.cross(rotation[:, axis], up))
        return rows

    def payload_torque(self, joint_positions, mass, com):
        """점질량 하나가 만드는 관절토크 [Nm] — 회귀 모델을 그대로 쓴다."""
        theta = np.array([mass, mass * com[0], mass * com[1], mass * com[2]])
        return self.regression_rows(joint_positions) @ theta


def solve(model, samples):
    """표본 [(q7, τ7)] 를 모아 페이로드와 관절 오프셋을 함께 푼다.

    반환: mass, com(3), bias(7), rms_before, rms_after, n.
    자세가 3개 미만이면 손목 각도 다양성이 모자라 com x/y 가 풀리지 않는다.
    """
    if len(samples) < 3:
        raise ValueError('표본이 3개 이상 필요하다 (자세를 다양하게).')

    residual = np.hstack([
        np.asarray(torque, dtype=float) - model.nominal_gravity(q)
        for q, torque in samples])
    payload_columns = np.vstack([model.regression_rows(q) for q, _ in samples])
    bias_columns = np.tile(np.eye(ARM_JOINT_COUNT), (len(samples), 1))
    design = np.hstack([payload_columns, bias_columns])

    theta, *_ = np.linalg.lstsq(design, residual, rcond=None)
    mass = float(theta[0])
    com = ([float(theta[1 + k] / mass) for k in range(3)]
           if abs(mass) > 1e-3 else [0.0, 0.0, 0.0])
    after = residual - design @ theta
    return {
        'mass': mass,
        'com': com,
        'bias': [float(v) for v in theta[4:]],
        'rms_before': float(np.sqrt(np.mean(residual ** 2))),
        'rms_after': float(np.sqrt(np.mean(after ** 2))),
        'n': len(samples),
        'condition': float(np.linalg.cond(design)),
    }
