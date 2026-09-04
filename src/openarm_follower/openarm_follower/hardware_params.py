"""follower.yaml 의 실측값을 생성된 URDF 의 하드웨어 블록에 싣는다.

`<ros2_control><hardware><param>` 를 emit 하는 xacro 는 `openarm_description` 안에 있는데,
이 workspace 는 그 리포를 업스트림에서 그대로 받아 쓰고 고치지 않는다. 그래서 값은 xacro 가
만들어 낸 문서에 붙인다 — 팔로워를 띄우는 launch 든 MoveIt demo 든, 실기로 갈 때 이 함수를
한 번 부르는 것이 하드웨어 블록에 실측값이 들어가는 유일한 경로다.
"""

import yaml

#: 로봇 전체에 하나인 값. yaml 최상위에서 온다.
ROBOT_PARAMS = ('gravity_comp', 'root_link', 'saturation_cap')

#: 팔마다 다른 값. yaml 의 arms.<arm> 에서 온다.
ARM_PARAMS = ('tip_link', 'payload_mass', 'payload_com', 'tau_bias')

#: 값을 받는 하드웨어 플러그인. 다른 블록(mock 등)은 건드리지 않는다.
PLUGIN = 'openarm_hardware/OpenArmHW'


def format_value(value):
    """OpenArmHW 가 읽는 모양으로 만든다.

    리스트는 플러그인의 istringstream 이 읽는 공백 구분 숫자로, 나머지는 소문자 문자열로
    (불리언은 "true"/"1"/"on" 과 비교된다).
    """
    if isinstance(value, (list, tuple)):
        return ' '.join(f'{float(v):.6g}' for v in value)
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def inject(document, config_path):
    """문서 안 OpenArmHW 블록마다 실측값을 `<param>` 으로 붙인다. 문서를 제자리에서 고친다.

    블록은 plugin 문자열로 고르므로 mock hardware 로 띄운 문서는 그대로 지나간다. 어느 팔의
    값을 실을지는 블록 자신의 `arm_prefix` 가 정하므로, 왼팔 블록에 오른팔 페이로드가 실릴 수
    없다.
    """
    with open(config_path, 'r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle) or {}
    arms = config.get('arms') or {}

    for hardware in document.getElementsByTagName('hardware'):
        plugins = hardware.getElementsByTagName('plugin')
        if not plugins or not plugins[0].firstChild:
            continue
        if plugins[0].firstChild.data.strip() != PLUGIN:
            continue

        existing = {}
        for node in hardware.getElementsByTagName('param'):
            if node.firstChild:
                existing[node.getAttribute('name')] = node.firstChild.data.strip()
        arm_config = arms.get(existing.get('arm_prefix', '').rstrip('_')) or {}

        values = {key: config[key] for key in ROBOT_PARAMS if key in config}
        values.update({key: arm_config[key]
                       for key in ARM_PARAMS if key in arm_config})

        for name, value in values.items():
            element = document.createElement('param')
            element.setAttribute('name', name)
            element.appendChild(document.createTextNode(format_value(value)))
            hardware.appendChild(element)
