"""udev 규칙 생성 순수 함수 테스트."""

from openarm_leader import udev


ATTRS = {
    'idVendor': '10c4',
    'idProduct': 'ea60',
    'serial': 'A1B2C3',
    'kernels': '1-2.3',
}


def test_build_rule_uses_serial_when_present():
    rule = udev.build_rule('left', ATTRS)
    assert 'ATTRS{serial}=="A1B2C3"' in rule
    assert 'KERNELS' not in rule
    assert 'SYMLINK+="openarm_leader_left"' in rule
    assert 'GROUP="dialout"' in rule


def test_build_rule_falls_back_to_kernels():
    attrs = dict(ATTRS, serial=None)
    rule = udev.build_rule('right', attrs)
    assert 'KERNELS=="1-2.3"' in rule
    assert 'serial' not in rule


def test_merge_rules_replaces_same_arm_only():
    left_old = udev.build_rule('left', dict(ATTRS, serial='OLD'))
    right = udev.build_rule('right', dict(ATTRS, serial='RIGHT'))
    existing = udev.merge_rules('', 'left', left_old)
    existing = udev.merge_rules(existing, 'right', right)

    left_new = udev.build_rule('left', ATTRS)
    merged = udev.merge_rules(existing, 'left', left_new)

    assert merged.count('openarm_leader_left') == 1
    assert 'ATTRS{serial}=="OLD"' not in merged
    assert 'ATTRS{serial}=="A1B2C3"' in merged
    assert 'ATTRS{serial}=="RIGHT"' in merged
    assert merged.startswith(udev.RULES_HEADER)
    assert merged.endswith('\n')


def test_resolve_conflicts_demotes_duplicate_serials():
    collected = {
        'left': dict(ATTRS, kernels='1-2'),
        'right': dict(ATTRS, kernels='1-3'),
    }
    resolved = udev.resolve_conflicts(collected)
    assert resolved['left']['serial'] is None
    assert resolved['right']['serial'] is None


def test_resolve_conflicts_keeps_distinct_serials():
    collected = {
        'left': dict(ATTRS),
        'right': dict(ATTRS, serial='OTHER'),
    }
    resolved = udev.resolve_conflicts(collected)
    assert resolved['left']['serial'] == 'A1B2C3'
    assert resolved['right']['serial'] == 'OTHER'
