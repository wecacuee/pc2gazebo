
def gazebo_generate_point_cloud(ingazebo):
    """
    Generate a point cloud from a husky in the gazebo model.
    """
    pass

def pc2gazebo():
    """
    Take a point cloud in
    and
    1. Detect classes and bounding boxes
    2. Generate gazebo models
    """
    pass


def demo_pc2gazebo(ingazebo="ext/gazebo-room-with-furniture/AtkHall6thFloorWithFurniture.world"):
    """
    Test pc2gazebo by:
    1. Generate point cloud from gazebo model
    2. Use pc2gazebo to convert back to gazebo model.
    3. Visualize generated gazebo model
    """
    pass


if __name__ == '__main__':
    demo_pc2gazebo()
