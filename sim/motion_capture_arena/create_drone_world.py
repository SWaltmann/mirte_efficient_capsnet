# This script serves to create the drone cage world for a gazebo 
# simulator. It allows to place some objects in the cage and vary their 
# position for training purposes. It is not very legible since I did not 
# care too much for that, yo ushould not have to interact directly with
# it (although you can if you want of course)

import numpy as np
import xml.etree.ElementTree as ET

def calculate_pose_and_size_3d(p1, p2, p3, p4):
    """ Calculate pose and size of rectangle from its corners """
    # Convert points to numpy arrays for easier vector calculations
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    p4 = np.array(p4)

    points = p1, p2, p3, p4

    # The next 20 lines are just to select the right vertices of the rectangle.
    # Someone smarter than I am can probably do it quicker ¯\_(ツ)_/¯
    vectors = np.zeros((6, 3))
    count = 0
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            vectors[count] = points[i] - points[j]
            count += 1

    lengths = np.linalg.norm(vectors, axis=1)

    side_length = np.sort(np.unique(lengths))

    if len(side_length) == 2:  # perfect square
        width, length = side_length[0], side_length[0]
    elif len(side_length) == 3:  # perfect rectangle
        length = side_length[0]
        width = side_length[1]
    else:  # not a rectangle
        raise f"This is not a rectangle: {points}"
    
    # Calculate the center of the rectangle (average of all four points)
    center = (p1 + p2 + p3 + p4) / 4
    
    print(f"Comparing {p1[2], p2[2], p3[2]}")
    if np.isclose(p1[2], p2[2]) and np.isclose(p2[2], p3[2]):  # means it is not a wall
        # Floor case: no roll or pitch, yaw is the angle between vec1 and the x-axis
        yaw = 0 #np.arctan2(vec1[1], vec1[0])  # Orientation around z-axis
        roll = 0
        pitch = 0
    else:
        # Lazily hard coded these angles :) Only straight boxes for now
        if np.isclose(p1[0], p2[0]) and np.isclose(p2[0], p3[0]):
            yaw = 0 # np.arctan2(vec1[1], vec1[0])  # Calculate yaw based on width vector
            roll = 0
            pitch = np.pi / 2 
        else:
            # Wall case: pitch by pi/2 to make it stand up, adjust yaw for rotation
            yaw = 0 # np.arctan2(vec1[1], vec1[0])  # Calculate yaw based on width vector
            roll = np.pi / 2  # Wall stands straight, need roll due to orientation
            pitch = 0  

    # The final pose consists of the center coordinates and the Euler angles (roll, pitch, yaw)
    pose = (center[0], center[1], center[2], roll, pitch, yaw)
    size = (width, length)



    return pose, size    


def create_rectangle(pose, size, thickness, suffix):
    """Create rectangle (useful for walls, ceiling)
    
    Taking into account that Gazebo uses the pose of an object to center 
    it on that pose. Not taking the thickness or direction of that 
    thickness into account
    """
    x, y, z, r, p, yaw = pose  # x, y, z, roll, pitch, yaw
    width, length = size

    # x, y, z = 0, 0, 0

    pose_str = f'{x} {y} {z} {r} {p} {yaw}'

    rect_name = f'rect_{suffix}'
    rectangle = ET.SubElement(world, 'model', name=rect_name)

    ET.SubElement(rectangle, 'static').text = 'true'

    # pose = ET.SubElement(rectangle, 'pose')
    # pose.text = pose_str 

    link = ET.SubElement(rectangle, 'link', name='link')

    # Add a collision element (for the floor's physical collisions)
    collision = ET.SubElement(link, 'collision', name='collision')
    collision_pose = ET.SubElement(collision, 'pose')
    collision_pose.text = pose_str  # Collision's position same as the pose above

    collision_geometry = ET.SubElement(collision, 'geometry')
    collision_box = ET.SubElement(collision_geometry, 'box')

    size = ET.SubElement(collision_box, 'size')
    size.text = f'{length} {width} {thickness}'  # 0.01 m thick floor

    visual = ET.SubElement(link, 'visual', name='visual')
    visual_pose = ET.SubElement(visual, 'pose')
    visual_pose.text = pose_str

    visual_geometry = ET.SubElement(visual, 'geometry')
    visual_box = ET.SubElement(visual_geometry, 'box')

    visual_size = ET.SubElement(visual_box, 'size')
    visual_size.text = size.text # Same size as the collision box

    # Add material for the black color
    visual_material = ET.SubElement(visual, 'material')
    visual_ambient = ET.SubElement(visual_material, 'ambient')
    visual_ambient.text = '0.0 0.0 0.0 1.0'  # Black color with full opacity
    visual_diffuse = ET.SubElement(visual_material, 'diffuse')
    visual_diffuse.text = '0.0 0.0 0.0 1.0'  # Black color with full opacity

# Create the root element for the SDF
sdf = ET.Element('sdf', version='1.7')

# Create a world element
world = ET.SubElement(sdf, 'world', name='drone_cage_world')

# Add physics settings to the world
physics = ET.SubElement(world, 'physics', name='1ms', type='ode')
max_step_size = ET.SubElement(physics, 'max_step_size')
max_step_size.text = '0.01'
real_time_factor = ET.SubElement(physics, 'real_time_factor')
real_time_factor.text = '1.0'

# Add plugins to the world
physics = ET.SubElement(world, 'plugin', 
                        filename="libignition-gazebo-physics-system.so", 
                        name="ignition::gazebo::systems::Physics")
commands = ET.SubElement(world, 'plugin',
                         filename="libignition-gazebo-user-commands-system.so",
                         name="ignition::gazebo::systems::UserCommands")
scene_bc = ET.SubElement(world, 'plugin',
                         filename="libignition-gazebo-scene-broadcaster-system.so",
                         name="ignition::gazebo::systems::SceneBroadcaster")

# Create GUI
gui = ET.SubElement(world, "gui", fullscreen="0")

# 3D scene
plugin = ET.SubElement(gui, "plugin", filename="GzScene3D", name="3D View")

ignition_gui = ET.SubElement(plugin, "ignition-gui")

title = ET.SubElement(ignition_gui, "title")
title.text = "3D View"
property = ET.SubElement(ignition_gui, "property", 
                         type="bool", key="showTitleBar")
property.text = "false"
property = ET.SubElement(ignition_gui, "property", 
                         type="string", key="state")
property.text = "docked"

engine = ET.SubElement(plugin, "engine")
engine.text = "ogre2"
scene = ET.SubElement(plugin, "scene")
scene.text = "scene"

ambient_light = ET.SubElement(plugin, "ambient_light")
ambient_light.text = "1.0 1.0 1.0"

background_color = ET.SubElement(plugin, "background_color")
background_color.text = "0.8 0.8 0.8"

camera_pose = ET.SubElement(plugin, "camera_pose")
camera_pose.text = "-6 0 6 0 0.5 0"

# World Control
plugin = ET.SubElement(gui, 'plugin', 
                       filename='WorldControl', name='World control')

ignition_gui = ET.SubElement(plugin, 'ignition-gui')
title = ET.SubElement(ignition_gui, 'title')
title.text = 'World control'

properties = [
    ('bool', 'showTitleBar', 'false'),
    ('bool', 'resizable', 'false'),
    ('double', 'height', '72'),
    ('double', 'width', '121'),
    ('double', 'z', '1'),
    ('string', 'state', 'floating')
]

for prop_type, key, value in properties:
    property_element = ET.SubElement(ignition_gui, 'property', 
                                     type=prop_type, key=key)
    property_element.text = value

anchors = ET.SubElement(ignition_gui, 'anchors', target='3D View')
ET.SubElement(anchors, 'line', own='left', target='left')
ET.SubElement(anchors, 'line', own='bottom', target='bottom')

ET.SubElement(plugin, 'play_pause').text = 'true'
ET.SubElement(plugin, 'step').text = 'true'
ET.SubElement(plugin, 'start_paused').text = 'true'
ET.SubElement(plugin, 'service').text = '/world/world_demo/control'
ET.SubElement(plugin, 'stats_topic').text = '/world/world_demo/stats'

# World statistics
plugin = ET.SubElement(gui, 'plugin', 
                       filename='WorldStats', name='World stats')

ignition_gui = ET.SubElement(plugin, 'ignition-gui')
title = ET.SubElement(ignition_gui, 'title')
title.text = 'World stats'

properties = [
    ('bool', 'showTitleBar', 'false'),
    ('bool', 'resizable', 'false'),
    ('double', 'height', '110'),
    ('double', 'width', '290'),
    ('double', 'z', '1'),
    ('string', 'state', 'floating')
]

for prop_type, key, value in properties:
    property_element = ET.SubElement(ignition_gui, 'property', 
                                     type=prop_type, key=key)
    property_element.text = value

anchors = ET.SubElement(ignition_gui, 'anchors', target='3D View')
ET.SubElement(anchors, 'line', own='right', target='right')
ET.SubElement(anchors, 'line', own='bottom', target='bottom')

ET.SubElement(plugin, 'sim_time').text = 'true'
ET.SubElement(plugin, 'real_time').text = 'true'
ET.SubElement(plugin, 'real_time_factor').text = 'true'
ET.SubElement(plugin, 'iterations').text = 'true'
ET.SubElement(plugin, 'topic').text = '/world/world_demo/stats'

# Entity tree
plugin = ET.SubElement(gui, "plugin", 
                       filename="EntityTree", name="Entity tree")


# Lighting
light = ET.SubElement(world, 'light', type='directional', name='sun')

ET.SubElement(light, 'cast_shadows').text = 'true'
ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
ET.SubElement(light, 'specular').text = '0.2 0.2 0.2 1'

attenuation = ET.SubElement(light, 'attenuation')
ET.SubElement(attenuation, 'range').text = '1000'
ET.SubElement(attenuation, 'constant').text = '0.9'
ET.SubElement(attenuation, 'linear').text = '0.01'
ET.SubElement(attenuation, 'quadratic').text = '0.001'

ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'

#################################
#                               # 
#   World building starts here  #
#                               #
#################################

# Size of the motion capture arena:
length = 16  # meter
height = 8  # meter
width = 8  # meter


arena_corners = np.zeros((8,3))

for i, corner in enumerate(arena_corners):
    # Simple way to get the coordinates of a rectangle of specified size
    # It cannot be rotated (by why would it)
    if i % 2 != 0:
        corner[0] = width / 2
    else:
        corner[0] = -width / 2

    if i in [1, 2, 5, 6]:
        corner[1] = length / 2
    else:
        corner[1] = -length / 2
    
    if i < 4 :
        corner[2] = 0
    else:
        corner[2] = height

# From this list, grab the 6 sides. Since it is constructed in a 
# specific way, we can assume which point goes where
arena_sides = [arena_corners[:4],
               arena_corners[4:],
               arena_corners[[0, 3, 4, 7]],
               arena_corners[[1, 2, 5, 6]],
               arena_corners[[1, 3, 5, 7]],
               arena_corners[[0, 2, 4, 6]]]

for i, side in enumerate(arena_sides):
    pose, size = calculate_pose_and_size_3d(*side)

    if i == 0:
        suffix = 'floor'
    elif i == 1:
        suffix = "ceiling"
    else:
        suffix = f"wall{i-2}"

    create_rectangle(pose, size, 0.01, suffix)

# # Create the model for the floor

# floor = ET.SubElement(world, 'model', name='floor')

# ET.SubElement(floor, 'static').text = 'true'

# pose = ET.SubElement(floor, 'pose')
# pose.text = '0 0 0 0 0 0' 

# link = ET.SubElement(floor, 'link', name='link')

# # Add a collision element (for the floor's physical collisions)
# collision = ET.SubElement(link, 'collision', name='collision')
# collision_pose = ET.SubElement(collision, 'pose')
# collision_pose.text = pose.text  # Collision's position same as the pose above

# collision_geometry = ET.SubElement(collision, 'geometry')
# collision_box = ET.SubElement(collision_geometry, 'box')

# size = ET.SubElement(collision_box, 'size')
# size.text = f'{length} {width} 0.01'  # 0.01 m thick floor

# visual = ET.SubElement(link, 'visual', name='visual')
# visual_pose = ET.SubElement(visual, 'pose')
# visual_pose.text = pose.text 

# visual_geometry = ET.SubElement(visual, 'geometry')
# visual_box = ET.SubElement(visual_geometry, 'box')

# visual_size = ET.SubElement(visual_box, 'size')
# visual_size.text = size.text # Same size as the collision box

# # Add material for the black color
# visual_material = ET.SubElement(visual, 'material')
# visual_ambient = ET.SubElement(visual_material, 'ambient')
# visual_ambient.text = '0.0 0.0 0.0 1.0'  # Black color with full opacity
# visual_diffuse = ET.SubElement(visual_material, 'diffuse')
# visual_diffuse.text = '0.0 0.0 0.0 1.0'  # Black color with full opacity

# Create walls:









# Tree structure and write to file
tree = ET.ElementTree(sdf)
ET.indent(tree, '    ')
tree.write('motion_capture_arena.sdf', 
           xml_declaration=True, encoding='utf-8', method="xml")

print("SDF file generated successfully!")
