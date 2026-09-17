"""URDF 생성과 씬 조립.

xacro 가 만든 한 팔짜리 URDF 를 MuJoCo 가 읽을 수 있게 고친 뒤, 같은 팔을 세 번
붙여 하나의 모델로 만든다. 붙일 때마다 접두사가 달라지므로 관절·물체 이름이 겹치지
않고, 세 팔은 서로 독립된 운동사슬이라 중력토크도 팔별로 따로 계산된다.
"""

import math
import subprocess
import xml.etree.ElementTree as ElementTree
import sys
from pathlib import Path

import mujoco

import config


def description_share():
    """openarm_description 의 share 경로. ROS 환경이 없으면 소스 경로로 떨어진다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('openarm_description'))
    except Exception:
        return config.DESCRIPTION_FALLBACK


def generate_urdf():
    """xacro 를 돌려 MuJoCo 용 URDF 를 만들고 그 경로를 돌려준다.

    마운트(몸통)는 xacro 가 bimanual 일 때만 만들어 준다. 그래서 양팔로 뽑은 다음 한쪽
    팔을 지운다 — 남는 것은 실물과 같은 마운트에 팔 하나가 달린 모습이다.

    MuJoCo 의 URDF 파서는 collada(.dae) 를 못 읽는데 openarm 의 visual 메시가 그
    형식이다. `discardvisual` 로 visual 을 버리면 STL 로 된 collision 메시만 남아
    그대로 그려진다. 관성은 URDF 값을 쓰므로 보이는 모양만 바뀐다.

    `fusestatic` 은 꺼 둔다. 켜 두면 고정 관절로 매달린 마운트가 world 에 녹아들어,
    팔을 여러 번 붙일 때 마운트가 따라오지 않는다.
    """
    share = description_share()
    xacro_path = share / config.XACRO_RELATIVE
    if not xacro_path.exists():
        sys.exit(f'xacro 를 찾지 못했습니다: {xacro_path}')

    args = [f'{key}:={value}' for key, value in config.XACRO_MAPPINGS.items()]
    xml = subprocess.run(['xacro', str(xacro_path), *args],
                         capture_output=True, text=True, check=True).stdout
    xml = xml.replace('package://openarm_description', str(share))

    robot = ElementTree.fromstring(xml)
    prune_arm(robot, f'openarm_{config.PRUNED_SIDE}_')
    compiler = ElementTree.SubElement(
        ElementTree.SubElement(robot, 'mujoco'), 'compiler')
    compiler.set('discardvisual', 'true')
    compiler.set('balanceinertia', 'true')
    compiler.set('fusestatic', 'false')
    compiler.set('strippath', 'false')

    config.GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    path = config.GENERATED_DIR / 'openarm_mujoco.urdf'
    path.write_bytes(ElementTree.tostring(robot, encoding='utf-8'))
    return path


def prune_arm(robot, prefix):
    """접두사가 붙은 링크와 관절을 통째로 지운다."""
    for element in list(robot):
        if element.tag in ('link', 'joint') and \
                element.get('name', '').startswith(prefix):
            robot.remove(element)


def add_backdrop(scene):
    """하늘과 바닥. 세 팔의 높이 차이를 눈으로 재도록 바닥에 격자를 깐다."""
    scene.add_texture(
        name='sky', type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        width=512, height=512, rgb1=[0.32, 0.38, 0.48], rgb2=[0.05, 0.06, 0.08])
    scene.add_texture(
        name='grid', type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        width=512, height=512, rgb1=[0.22, 0.23, 0.25], rgb2=[0.28, 0.29, 0.32])
    material = scene.add_material(name='floor', texrepeat=[6, 6], reflectance=0.1)
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'grid'

    scene.worldbody.add_light(pos=[0, 0, 4], dir=[0, 0, -1],
                              type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    scene.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[6, 6, 0.1],
        pos=[0, 0, 0],
        material='floor',
    )


def build_scene(urdf_path):
    """팔 셋을 나란히 붙인 모델을 컴파일한다."""
    scene = mujoco.MjSpec()
    scene.option.timestep = config.TIMESTEP
    scene.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    scene.visual.global_.offwidth = config.OFFSCREEN_SIZE[0]
    scene.visual.global_.offheight = config.OFFSCREEN_SIZE[1]

    add_backdrop(scene)

    span = (len(config.ARMS) - 1) * config.ARM_SPACING
    for index, (name, _gain, _label, rgba) in enumerate(config.ARMS):
        arm = mujoco.MjSpec.from_file(str(urdf_path))
        for geom in arm.geoms:
            # 마운트는 셋이 같은 회색이고, 팔만 계수별 색으로 칠한다.
            if f'_{config.ARM_SIDE}_' in geom.name or not geom.name:
                geom.rgba = rgba
            else:
                geom.rgba = [0.40, 0.41, 0.44, 1.0]
        origin = [0.0, index * config.ARM_SPACING - span / 2, 0.0]
        yaw = math.radians(config.MOUNT_YAW_DEG)
        frame = scene.worldbody.add_frame(
            pos=origin, quat=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
        frame.attach_body(arm.worldbody.first_body(), f'{name}_', '')

    model = scene.compile()
    model.dof_damping[:] = config.JOINT_DAMPING
    if not config.ENABLE_CONTACTS:
        model.geom_contype[:] = 0
        model.geom_conaffinity[:] = 0
    return model
