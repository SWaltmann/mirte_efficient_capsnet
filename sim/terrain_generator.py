# This script serves to create a heightmap for a gazebo simulation. This
# is simply a grayscale .png image, where black is the lowest point
# and white the highest. The goal of the script is to create a relatively
# flat ground heighmap with some gradual hills, like a forest. Importantly,
# these should be no steep hills or holes so the entire floor is accessible 
# for a robot and we do not need to check of tree can or cannot grow in 
# certain places.



from PIL import Image

heightmap = Image.new(mode='L',  # mode 'L' is 8 bit grayscale image
                      size = (100,100),
                      color = 128)

heightmap.save('heightmap.png')

heightmap.show()