# This script serves to create the drone cage world for a gazebo 
# simulator. It allows to place some objects in the cage and vary their 
# position for training purposes. It is not very legible since I did not 
# care too much for that, yo ushould not have to interact directly with
# it (although you can if you want of course)

import xml.etree.ElementTree as ET

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


# Create the light element
light = ET.SubElement(world, 'light', type='directional', name='sun')

# Set light properties
ET.SubElement(light, 'cast_shadows').text = 'true'
ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
ET.SubElement(light, 'specular').text = '0.2 0.2 0.2 1'

# Create the attenuation subelement
attenuation = ET.SubElement(light, 'attenuation')
ET.SubElement(attenuation, 'range').text = '1000'
ET.SubElement(attenuation, 'constant').text = '0.9'
ET.SubElement(attenuation, 'linear').text = '0.01'
ET.SubElement(attenuation, 'quadratic').text = '0.001'

ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'



# Tree structure and write to file
tree = ET.ElementTree(sdf)
ET.indent(tree, '    ')
tree.write('motion_capture_arena.sdf', 
           xml_declaration=True, encoding='utf-8', method="xml")

print("SDF file generated successfully!")