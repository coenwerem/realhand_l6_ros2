# Copyright 2026 Clinton Enwerem
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Process the hand xacro for every side and hardware option and check the tree."""

# The joint and sensor names the driver expects come from realhand_hardware's
# model table, so a rename on one side without the other fails here.

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

ACTUATED = [
    'thumb_cmc_pitch', 'thumb_cmc_yaw', 'index_mcp_pitch', 'middle_mcp_pitch',
    'ring_mcp_pitch', 'pinky_mcp_pitch']
MIMIC = ['thumb_ip', 'index_dip', 'middle_dip', 'ring_dip', 'pinky_dip']
FINGERS = ['thumb', 'index', 'middle', 'ring', 'pinky']


def render(**args):
    # The source tree, so the test does not need its own package on the
    # ament index.
    path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'realhand_l6.urdf.xacro')
    cmd = ['xacro', path] + [f'{k}:={v}' for k, v in args.items()]
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout


@pytest.mark.parametrize('side', ['right', 'left'])
@pytest.mark.parametrize('prefix', ['', 'rh_'])
def test_kinematics(side, prefix):
    urdf = render(side=side, prefix=prefix, hardware='none')
    root = ET.fromstring(urdf)
    joints = {j.attrib['name']: j for j in root.findall('joint')}
    for name in ACTUATED:
        assert joints[prefix + name].attrib['type'] == 'revolute'
        assert joints[prefix + name].find('mimic') is None
    for name in MIMIC:
        mimic = joints[prefix + name].find('mimic')
        assert mimic is not None
        assert mimic.attrib['joint'].startswith(prefix)
    for finger in FINGERS:
        assert joints[prefix + finger + '_pad_joint'].attrib['type'] == 'fixed'
    assert joints[prefix + 'hand_to_tcp'].attrib['type'] == 'fixed'
    yaw_axis = joints[prefix + 'thumb_cmc_yaw'].find('axis').attrib['xyz'].split()
    assert float(yaw_axis[2]) == (-1.0 if side == 'right' else 1.0)
    for link in root.findall('link'):
        mesh = link.find('visual/geometry/mesh')
        if mesh is not None:
            assert f'/meshes/l6_{side}/' in mesh.attrib['filename']
    # urdfdom accepts the tree.
    subprocess.run(['check_urdf', '/dev/stdin'], input=urdf, check=True,
                   capture_output=True, text=True)


@pytest.mark.parametrize('hardware', ['mock', 'real'])
def test_ros2_control_block(hardware):
    urdf = render(side='right', prefix='right_', hardware=hardware,
                  can_interface='can3', can_id='0x2a')
    root = ET.fromstring(urdf)
    block = root.find('ros2_control')
    assert block is not None
    plugin = block.find('hardware/plugin').text
    if hardware == 'mock':
        assert plugin == 'mock_components/GenericSystem'
    else:
        assert plugin == 'realhand_hardware/RealHandSystem'
        params = {p.attrib['name']: p.text for p in block.findall('hardware/param')}
        assert params['model'] == 'L6'
        assert params['hand_side'] == 'right'
        assert params['joint_prefix'] == 'right_'
        assert params['can_interface'] == 'can3'
        assert params['can_id'] == '0x2a'
    joints = {j.attrib['name']: j for j in block.findall('joint')}
    for name in ACTUATED:
        j = joints['right_' + name]
        commands = {c.attrib['name'] for c in j.findall('command_interface')}
        assert commands == {'position', 'speed', 'torque'}
    for name in MIMIC:
        assert joints['right_' + name].find('command_interface') is None
    sensors = {s.attrib['name'] for s in block.findall('sensor')}
    assert sensors == {'right_tactile_' + f for f in FINGERS}


def test_setpoints_can_be_left_out():
    root = ET.fromstring(render(hardware='real', setpoints='false'))
    for j in root.find('ros2_control').findall('joint'):
        commands = {c.attrib['name'] for c in j.findall('command_interface')}
        assert commands in ({'position'}, set())


def test_no_hardware_block_by_default():
    root = ET.fromstring(render())
    assert root.find('ros2_control') is None
