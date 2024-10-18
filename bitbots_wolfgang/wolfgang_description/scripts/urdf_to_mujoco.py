#!/usr/bin/env python

import os
import sys
import tempfile
import mujoco
from lxml import etree

# This script converts a URDF file to a MuJoCo XML model file.

INPUT_URDF = '/home/florian/Projekt/bitbots/bitbots_main/bitbots_wolfgang/wolfgang_description/urdf/robot.urdf'

ACTUATOR_TYPES = ['mx106', 'mx64', 'xh-540']

ACTUATOR_MAPPING = {
    'HeadPan': 'mx64',
    'HeadTilt': 'mx64',
    'LShoulderPitch': 'mx106',
    'LShoulderRoll': 'mx106',
    'LElbow': 'mx64',
    'LAnklePitch': 'mx106',
    'LAnkleRoll': 'mx106',
    'LHipYaw': 'mx106',
    'LHipRoll': 'mx106',
    'LHipPitch': 'mx106',
    'LKnee': 'xh-540',
    'RShoulderPitch': 'mx106',
    'RShoulderRoll': 'mx106',
    'RElbow': 'mx64',
    'RAnklePitch': 'mx106',
    'RAnkleRoll': 'mx106',
    'RHipYaw': 'mx106',
    'RHipRoll': 'mx106',
    'RHipPitch': 'mx106',
    'RKnee': 'xh-540',
} # TODO verify the mapping

ACTUATOR_DEFAULTS = {
    'mx106': {
        "joint": {
            "damping": "1.23",
            "armature": "0.045",
            "frictionloss": "2.55",
            "limited": "true"
        },
        "position": {  # This is for the position controller
            "kp": "21.1",
            "ctrlrange": "-3.141592 3.141592",
            "forcerange": "-5 5"
        }
    },
    'mx64': {
        "joint": {
            "damping": "0.65",
            "armature": "0.045",
            "frictionloss": "1.73",
            "limited": "true"
        },
        "position": {  # This is for the position controller
            "kp": "21.1",
            "ctrlrange": "-3.141592 3.141592",
            "forcerange": "-5 5"
        }
    },
    'xh-540': {
        "joint": {
            "damping": "2.92",
            "armature": "0.045",
            "frictionloss": "1.49",
            "limited": "true"
        },
        "position": {  # This is for the position controller
            "kp": "21.1",
            "ctrlrange": "-3.141592 3.141592",
            "forcerange": "-5 5"
        }
    }
}


def main():
    # Load the urdf file
    if not os.path.exists(INPUT_URDF):
        print('Error: URDF file not found:', INPUT_URDF)
        sys.exit(1)
    urdf_tree = etree.parse(INPUT_URDF)

    # Add the mujoco tag to the URDF file to make it compatible with mujoco
    mujoco_tag = etree.Element('mujoco')
    mujoco_tag.append(etree.Element('compiler', discardvisual='false', meshdir=os.path.dirname(INPUT_URDF)))
    urdf_tree.getroot().append(mujoco_tag)

    # Render the URDF file as a string
    urdf_string = etree.tostring(urdf_tree, pretty_print=True)

    # Load the URDF file into mujoco
    model = mujoco.MjModel.from_xml_string(urdf_string)

    # Save model as XML (temporary file)
    temp_xml_file_path = tempfile.mktemp()
    mujoco.mj_saveLastXML(temp_xml_file_path, model)

    # Load the XML file into an etree
    tree = etree.parse(temp_xml_file_path)

    # Apply some modifications / fixes

    # Move everything in the worldbody into a new body called torso (also add freejoint and light)   # TODO investigate why the torso is not the root body
    worldbody = tree.find('.//worldbody')
    torso = etree.Element('body', name='torso', pos="0 0 0.4274", quat="0.999 0.0 0.05 0.0") # TODO check if pos is correct
    for child in worldbody.getchildren():
        torso.append(child)
    torso.append(etree.Element('freejoint'))
    worldbody.clear()
    worldbody.append(torso)
    worldbody.append(etree.Element('light', name='spotlight', mode='targetbodycom', target='torso', pos='0 -1 2'))


    # Assign classes to all geometries
    # Find visual elements, meaning geometries with contype="0" conaffinity="0" group="1" density="0"
    for geom in tree.findall('.//geom[@contype="0"][@conaffinity="0"][@group="1"][@density="0"]'):
        # Remove the attributes
        geom.attrib.pop('contype', None)
        geom.attrib.pop('conaffinity', None)
        geom.attrib.pop('group', None)
        geom.attrib.pop('density', None)

        # Also remove the rgba attribute
        geom.attrib.pop('rgba', None)

        # Assign the class attribute
        geom.attrib['class'] = 'visual'

    # Find geometries that don't have a class yet and assign them to the collision class
    for geom in tree.xpath('.//geom[not(@class)]'):
        geom.attrib['class'] = 'collision'

    # Add defaults

    defaults = etree.fromstring("""
    <default>
        <site group="5" type="sphere"/>
        <default class="collision">
            <geom group="3"/>
        </default>
        <default class="visual">
            <geom material="black" contype="0" conaffinity="0" group="2"/>
        </default>
    </default>
    """)

    # Add actuator and joint defaults
    assert set(ACTUATOR_DEFAULTS.keys()) == set(ACTUATOR_TYPES)
    for actuator_type, actuator_defaults in ACTUATOR_DEFAULTS.items():
        default = etree.Element('default', **{'class': actuator_type})
        default.extend([
            etree.Element('joint', **actuator_defaults['joint']),
            etree.Element('position', **actuator_defaults['position'])
        ])
        defaults.append(default)
    tree.getroot().insert(0, defaults)

    # Remove meshdir attribute from compiler tag
    tree.find('.//compiler').attrib.pop('meshdir', None)

    # Add 'black' material to assets
    tree.find('.//asset').append(etree.Element('material', name='black', rgba='0.2 0.2 0.2 1'))

    # Remove damping and frictionloss from all joints in the worldbody
    for joint in tree.findall('.//worldbody/.//joint'):
        # Remove the attributes
        joint.attrib.pop('damping', None)
        joint.attrib.pop('frictionloss', None)
        joint.attrib.pop('limited', None)
        # Add class based on the actuator type
        joint.attrib['class'] = ACTUATOR_MAPPING[joint.attrib['name']]

    # Add actuators to the top level
    actuator = etree.Element('actuator')
    for joint in tree.findall('.//worldbody/.//joint'):
        actuator.append(etree.Element('position', **{'joint': joint.attrib['name'], 'name': joint.attrib['name'], 'class': ACTUATOR_MAPPING[joint.attrib['name']] }))
    tree.getroot().append(actuator)

    # Save the XML file with pretty formatting
    output_filename = os.path.splitext(INPUT_URDF)[0] + '.xml'
    tree.write(output_filename, pretty_print=True)
    print('Saved MuJoCo model to', output_filename)

if __name__ == '__main__':
    main()
