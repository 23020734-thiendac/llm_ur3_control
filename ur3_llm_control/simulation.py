# Build Gazebo geometry and robot description from the installed Humble UR model.
from xml.etree import ElementTree as ET


def world_sdf(scene):
    def box(name, xyz, size, color, collision=True):
        pose = ' '.join(map(str, xyz)) + ' 0 0 0'
        dimensions = ' '.join(map(str, size))
        rgba = ' '.join(map(str, color))
        geometry = f'<geometry><box><size>{dimensions}</size></box></geometry>'
        solid = f'<collision name="collision">{geometry}</collision>' if collision else ''
        return f'''<model name="{name}"><static>true</static><pose>{pose}</pose>
          <link name="link">{solid}<visual name="visual">{geometry}
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
          </visual></link></model>'''

    models = [box('floor', [0, 0, -0.07], [3, 3, 0.02], [0.35, 0.38, 0.4, 1]),
              box('work_table', scene['table']['center'], scene['table']['size'], [0.5, 0.36, 0.22, 1])]
    for name, data in scene['objects'].items():
        models.append(box(name, data['position'], [scene['cube_size']]*3, data['color']))
    plus = scene.get('plus_object')
    if plus:
        # Keep the optional plus object on the table at its parking point.
        # RobotSkills moves it into the selected A/B/C zone when a/b/c is pressed.
        models.append(box(plus['name'], plus['parking_position'],
                          [scene['cube_size']]*3, [0.95, 0.35, 0.05, 1.0]))
    top = scene['table']['center'][2] + scene['table']['size'][2]/2
    if plus:
        models.append(box('temporary_zone',
                          [plus['temporary_zone'][0], plus['temporary_zone'][1], top + 0.0005],
                          [0.075, 0.075, 0.001], [0.95, 0.45, 0.05, 1], collision=False))
    for index, (name, xyz) in enumerate(scene['zones'].items()):
        models.append(box(name, [xyz[0], xyz[1], top + 0.0005], [0.075, 0.075, 0.001],
                          [0.2, 0.65, 0.35, 1], collision=False))
        patterns = [('01110', '10001', '11111', '10001', '10001'),
                    ('11110', '10001', '11110', '10001', '11110'),
                    ('01111', '10000', '10000', '10000', '01111')]
        for row, pixels in enumerate(patterns[index]):
            for col, bit in enumerate(pixels):
                if bit == '1':
                    models.append(box(f'{name}_label_{row}_{col}',
                                      [xyz[0]+0.025+(4-row)*0.003, xyz[1]+(2-col)*0.003, top+0.002],
                                      [0.003, 0.003, 0.001], [1, 1, 1, 1], collision=False))
    return '''<?xml version="1.0"?><sdf version="1.7"><world name="%s">
      <physics name="physics" type="ignored"><max_step_size>0.002</max_step_size>
        <real_time_factor>1</real_time_factor></physics>
      <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
      <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
      <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
      <light type="directional" name="sun"><pose>0 0 5 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular><direction>-0.5 0.1 -1</direction></light>
      <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.2 0.23 0.28 1</background></scene>
      %s
    </world></sdf>''' % (scene['world_name'], '\n'.join(models))


def add_parallel_gripper(urdf):
    # Add a controllable two-finger parallel gripper to the UR3 tool frame.
    root = ET.fromstring(urdf)
    for child in list(root):
        if child.attrib.get('name') in ('ground_plane', 'ground_plane_joint'):
            root.remove(child)
    control = root.find('ros2_control')
    if control is None:
        raise ValueError('URDF không có ros2_control để thêm gripper')
    for name in ('gripper_left_joint', 'gripper_right_joint'):
        joint = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(joint, 'command_interface', name='position')
        ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(joint, 'state_interface', name='velocity')
    extra = ET.fromstring('''<robot>
      <!-- Elongated jaw carrier covers both finger travel endpoints. -->
      <link name="gripper_base"><inertial><origin xyz="0 0 0.015"/><mass value="0.08"/>
        <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/></inertial>
        <visual><origin xyz="0 0 0.015"/><geometry><box size="0.035 0.095 0.03"/></geometry>
          <material name="gripper_base_material"><color rgba="0.12 0.12 0.14 1"/></material></visual>
        <collision><origin xyz="0 0 0.015"/><geometry><box size="0.035 0.095 0.03"/></geometry></collision></link>
      <joint name="gripper_mount" type="fixed"><parent link="tool0"/><child link="gripper_base"/><origin xyz="0 0 0"/></joint>
      <link name="gripper_left_finger"><inertial><origin xyz="0 0 0.02"/><mass value="0.025"/>
        <inertia ixx="0.00002" iyy="0.00002" izz="0.00002" ixy="0" ixz="0" iyz="0"/></inertial>
        <visual><origin xyz="0 0 0.02"/><geometry><box size="0.012 0.010 0.04"/></geometry>
          <material name="finger_material"><color rgba="0.2 0.22 0.25 1"/></material></visual>
        <collision><origin xyz="0 0 0.02"/><geometry><box size="0.012 0.010 0.04"/></geometry></collision></link>
      <joint name="gripper_left_joint" type="prismatic"><parent link="gripper_base"/><child link="gripper_left_finger"/>
        <origin xyz="0 0.035 0.03"/><axis xyz="0 1 0"/><limit lower="-0.012" upper="0.0" effort="20" velocity="0.2"/></joint>
      <link name="gripper_right_finger"><inertial><origin xyz="0 0 0.02"/><mass value="0.025"/>
        <inertia ixx="0.00002" iyy="0.00002" izz="0.00002" ixy="0" ixz="0" iyz="0"/></inertial>
        <visual><origin xyz="0 0 0.02"/><geometry><box size="0.012 0.010 0.04"/></geometry>
          <material name="finger_material"><color rgba="0.2 0.22 0.25 1"/></material></visual>
        <collision><origin xyz="0 0 0.02"/><geometry><box size="0.012 0.010 0.04"/></geometry></collision></link>
      <joint name="gripper_right_joint" type="prismatic"><parent link="gripper_base"/><child link="gripper_right_finger"/>
        <origin xyz="0 -0.035 0.03"/><axis xyz="0 -1 0"/><limit lower="-0.012" upper="0.0" effort="20" velocity="0.2"/></joint>
      <link name="grasp_link"/><joint name="grasp_tip" type="fixed"><parent link="gripper_base"/><child link="grasp_link"/>
        <origin xyz="0 0 0.03"/></joint>
    </robot>''')
    root.extend(list(extra))
    return ET.tostring(root, encoding='unicode')


def add_parallel_gripper_semantic(srdf):
    root = ET.fromstring(srdf)
    root.find("group[@name='ur_manipulator']/chain").set('tip_link', 'grasp_link')
    for link in ('gripper_base', 'gripper_left_finger', 'gripper_right_finger'):
        for other in ('tool0', 'flange', 'wrist_3_link', 'grasp_link'):
            ET.SubElement(root, 'disable_collisions', link1=link, link2=other, reason='Adjacent')
    ET.SubElement(root, 'disable_collisions', link1='gripper_base', link2='gripper_left_finger', reason='Adjacent')
    ET.SubElement(root, 'disable_collisions', link1='gripper_base', link2='gripper_right_finger', reason='Adjacent')
    # Keep the UR3 shoulder/base adjacent pair disabled explicitly. Some
    # Humble ur_moveit_config revisions omit this entry after xacro expansion.
    ET.SubElement(root, 'disable_collisions', link1='base_link_inertia',
                  link2='shoulder_link', reason='Adjacent')
    return ET.tostring(root, encoding='unicode')

