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

"""Render the SRDF macro and check it against the description's URDF names."""

import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

HERE = os.path.dirname(__file__)
MIMIC = {'thumb_ip': ('thumb_cmc_pitch', 1.83), 'index_dip': ('index_mcp_pitch', 0.89),
         'middle_dip': ('middle_mcp_pitch', 0.89), 'ring_dip': ('ring_mcp_pitch', 0.89),
         'pinky_dip': ('pinky_mcp_pitch', 0.89)}


def render_srdf(prefix=''):
    path = os.path.join(HERE, '..', 'srdf', 'realhand_l6.srdf.xacro')
    out = subprocess.run(['xacro', path, f'prefix:={prefix}'], check=True,
                         capture_output=True, text=True).stdout
    return ET.fromstring(out)


def urdf_names(prefix=''):
    share = get_package_share_directory('realhand_l6_description')
    path = os.path.join(share, 'urdf', 'realhand_l6.urdf.xacro')
    out = subprocess.run(['xacro', path, f'prefix:={prefix}'], check=True,
                         capture_output=True, text=True).stdout
    root = ET.fromstring(out)
    links = {link.attrib['name'] for link in root.findall('link')}
    joints = {j.attrib['name'] for j in root.findall('joint')}
    return links, joints


@pytest.mark.parametrize('prefix', ['', 'rh_'])
def test_srdf_names_exist_in_urdf(prefix):
    srdf = render_srdf(prefix)
    links, joints = urdf_names(prefix)
    for group in srdf.findall('group'):
        for j in group.findall('joint'):
            assert j.attrib['name'] in joints
        for chain in group.findall('chain'):
            assert chain.attrib['base_link'] in links
            assert chain.attrib['tip_link'] in links
    for pair in srdf.findall('disable_collisions'):
        assert pair.attrib['link1'] in links
        assert pair.attrib['link2'] in links
    for pj in srdf.findall('passive_joint'):
        assert pj.attrib['name'] in joints
    # Standalone, so no end effector element and no parent group.
    assert srdf.find('end_effector') is None


def test_group_states_respect_mimic_ratios():
    srdf = render_srdf()
    states = {s.attrib['name']: s for s in srdf.findall('group_state')}
    assert set(states) == {'open', 'pinch', 'power'}
    for state in states.values():
        values = {j.attrib['name']: float(j.attrib['value']) for j in state.findall('joint')}
        for mimic, (parent, ratio) in MIMIC.items():
            assert abs(values[mimic] - values[parent] * ratio) < 2e-3


def test_hand_group_is_all_fingers():
    srdf = render_srdf()
    hand = [g for g in srdf.findall('group') if g.attrib['name'] == 'hand'][0]
    subs = {g.attrib['name'] for g in hand.findall('group')}
    assert subs == {'thumb', 'index', 'middle', 'ring', 'pinky'}


def test_end_effector_with_parent_group():
    path = os.path.join(HERE, '..', 'srdf', 'realhand_l6_macro.srdf.xacro')
    wrapper = f"""<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="t">
      <xacro:include filename="{path}"/>
      <xacro:realhand_l6_srdf prefix="rh_" parent_group="arm"/>
    </robot>"""
    tmp = os.path.join(HERE, '.wrapper.srdf.xacro')
    with open(tmp, 'w') as f:
        f.write(wrapper)
    try:
        out = subprocess.run(['xacro', tmp], check=True, capture_output=True, text=True).stdout
    finally:
        os.remove(tmp)
    ee = ET.fromstring(out).find('end_effector')
    assert ee.attrib['group'] == 'rh_hand'
    assert ee.attrib['parent_link'] == 'rh_hand_base_link'
    assert ee.attrib['parent_group'] == 'arm'
