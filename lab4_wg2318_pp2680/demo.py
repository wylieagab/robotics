from __future__ import division
import pybullet as p
import pybullet_data
import numpy as np
import time
import argparse
from random import seed
from random import random
from random import randint
import math
import sys

seed(1)
UR5_JOINT_INDICES = [0, 1, 2]
step_size = 0.05
bias = 5


def set_joint_positions(body, joints, values):
    """
    Given the values, sets the robot arm/joints to the position value. 
    """
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


def draw_sphere_marker(position, radius, color):
    """
    Plot dots given the position, radius and color. 
    """
    vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
    marker_id = p.createMultiBody(
        basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id
    )
    return marker_id


def remove_marker(marker_id):
    """
    Removes the marker given the id. 
    """
    p.removeBody(marker_id)


def get_args():
    """
    Given argument, calls the right path finding function.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--birrt", action="store_true", default=False)
    parser.add_argument("--smoothing", action="store_true", default=False)
    args = parser.parse_args()
    return args


##################################################################################################################
def getEndEffectorState(conf):
    """
    Moves the robot arm to a given configuration and then returns the 
    cartesian coordinates i.e position for the robot end effector or
    head of the robot. 
    """
    set_joint_positions(ur5, UR5_JOINT_INDICES, conf)
    return p.getLinkState(ur5, 3)[0]


class Tree:
    """
    Tree Class Declaration
    """

    def __init__(self):
        self.vertices = []
        self.edges = dict()
        self.N = 200


def genConfRandom(min_val, max_val):
    """
    Generates a random configuration between a min and max value.
    """
    rand_value = random()
    value = min_val + (rand_value * (max_val - min_val))
    return value


def distance_euclid(p1, p2):
    """
    Given two points, returns the euclidean distance. 
    """
    distance = math.sqrt(
        (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2
    )
    return distance


def nearest_n(vertices, q_rand):
    """
    Finds nearest neighbor(existing node) to the randomize configuration. 
    """
    q_near = (float("inf"), float("inf"), float("inf"))
    min_distance = float("inf")

    for vertex in vertices:

        distance = distance_euclid(q_rand, vertex)

        if distance < min_distance:
            min_distance = distance
            q_near = vertex

    return q_near


def progress(q_near, dest_conf, step = step_size):
    """    
    Progresses a new configuration given a nearest neighbor and randomize configuration.
    """
    try:
        vector = (
            dest_conf[0] - q_near[0],
            dest_conf[1] - q_near[1],
            dest_conf[2] - q_near[2],
        )

        vector_distance = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)

        unit_vector = (
            vector[0] / vector_distance,
            vector[1] / vector_distance,
            vector[2] / vector_distance,
        )

        # point = distance * unit + current_point
        q_new = (
            (step * unit_vector[0] + q_near[0]),
            (step * unit_vector[1] + q_near[1]),
            (step * unit_vector[2] + q_near[2]),
        )
        return q_new
    except:  # Zero Division Exception
        return None


def close_to_target(q, target):
    """
    Checks whether the configuration is within a step_size of the goal/start.
    """
    if distance_euclid(q, target) <= step_size:
        return True
    else:
        return False


def update_tree(T, existing_node, new_node, color):
    """
    Connect new node to Tree and draws a line.
    """
    # Add new node to vertices
    T.vertices.append(new_node)

    # Add new edge to edges
    if existing_node not in T.edges:
        T.edges[existing_node] = [new_node]
    else:
        T.edges[existing_node].append(new_node)

    # Compute positions by moving the arm
    exist_position = getEndEffectorState(existing_node)
    new_position = getEndEffectorState(new_node)

    # Draws a green/blue line from the existing node to new node
    if color == "green":
        # Green
        p.addUserDebugLine(exist_position, new_position, [0, 1, 0])
    else:
        # Blue
        p.addUserDebugLine(exist_position, new_position, [0, 0, 1])


def extend_rrt(T, q_rand, goal, color="green"):
    """
    Given the tree and randomize conf, try to connect it to the tree.
    And acknowledges the goal while connecting new nodes.  
    """
    # Find the nearest configuration
    q_near = nearest_n(T.vertices, q_rand)

    # Progress a new configuration that is within a step-size
    q_new = progress(q_near, q_rand)

    if q_new != None:
        if not collision_fn(q_new):
            # Add new node to Tree
            update_tree(T, q_near, q_new, color)

            # For every newly added node, check if it is within the step_size of the goal
            if close_to_target(q_new, goal):
                # Add goal to Tree
                update_tree(T, q_new, goal, color)
                # Return the goal and will terminate the build algorithm
                return goal

            # Just return the new node
            return q_new

    # Reached collision, return None and will be skipped.
    return None


def random_config(target):
    """
    Return a random configuration that is 5% biased sampling.
    """
    if randint(1, 100) <= bias:
        return target
    else:
        return (
            genConfRandom(-2 * math.pi, 2 * math.pi),
            genConfRandom(-2 * math.pi, 2 * math.pi),
            genConfRandom(-2 * math.pi, 2 * math.pi)
        )


def build_RRT():
    """
    Builds RRT algorithm 
    """
    T = Tree()
    T.vertices.append(start_conf)
    for i in range(0, T.N):
        # Randomize configuration in free space, must be 5% biased towards goal
        q_rand = random_config(goal_conf)

        # Should terminate if the goal is in the tree because we have found a path
        new_config = extend_rrt(T, q_rand, goal_conf)

        if new_config == goal_conf:
            break
    return T


def DFS(T, start, end):
    reverse_tree = dict()
    for head in T.edges:
        for tail in T.edges[head]:
            if tail not in reverse_tree:
                reverse_tree[tail] = [head]
            else:
                reverse_tree[tail].append(head)

    # Backtrack a path from the goal to the starting point
    if end in reverse_tree:

        # Keep tracing until the pointer is at the starting configuration
        stack = [end]
        while stack[-1] is not start:
            # Only one config
            if len(reverse_tree[stack[-1]]) == 1:
                # Adds the only node to the stack
                stack.append(reverse_tree[stack[-1]][0])
            # Dead End
            elif len(reverse_tree[stack[-1]]) == 0:
                # Remove from stack
                stack.pop()
                # Remove edge from Tree, most likely the first one
                reverse_tree[stack[-1]][1:]
            # More than one config
            elif len(reverse_tree[stack[-1]]) > 1:
                # Push the first edge
                stack.append(reverse_tree[stack[-1]][0])

        # Reverse the path of configurations
        return stack[::-1]
    else:
        # Goal doesn't exist in the Tree
        return None


def rrt():
    ###############################################
    # TODO your code to implement the rrt algorithm
    ###############################################
    """
    Build Path Configuration and return if goal is reached.
    We can do DFS because there is only one path from start to goal.
    """
    # Build a tree that reaches to the goal
    tree = build_RRT()
    return DFS(tree, start_conf, goal_conf)


def build_biRRT():
    """
    Builds biRRT algorithm 
    """
    # Create two empty trees with the starting configuration or goal
    T1 = Tree()
    T1.vertices.append(start_conf)
    T2 = Tree()
    T2.vertices.append(goal_conf)

    # Alternate between T1 and T2
    # Break this loop once you get a connecting node
    switch = True
    connect_config = (0, 0, 0)
    for i in range(0, T1.N):

        if switch:  # Randomize T1

            # Randomize configuration in free space, must be 5% biased towards goal
            q_rand_1 = random_config(goal_conf)
            new_config_1 = extend_rrt(T1, q_rand_1, goal_conf)
            """
            What if it reaches the goal before T2 reaches it?
            """

            # Extends T2 towards new configuration
            if new_config_1 != None:
                # Biased towards T1's new configuration
                new_config_2 = extend_rrt(T2, new_config_1, new_config_1, "blue")
                if new_config_2 == new_config_1:
                    connect_config = new_config_2
                    break

        else:  # Randomize T2

            # Randomize configuration in free space, must be 5% biased towards start
            q_rand_2 = random_config(start_conf)
            new_config_2 = extend_rrt(T2, q_rand_2, start_conf, "blue")
            """
            What if it reaches start before T1 reaches it?
            """

            # Extends T1 towards new configuration
            if new_config_2 != None:
                # Biased towards T2's new configuration
                new_config_1 = extend_rrt(T1, new_config_2, new_config_2)
                if new_config_1 == new_config_2:
                    connect_config = new_config_1
                    break
        switch = not switch
    return T1, T2, connect_config


def birrt():
    #################################################
    # TODO your code to implement the birrt algorithm
    #################################################
    """
    Build Path Configuration with two trees and try to connect them
    """
    T1, T2, connecting_node = build_biRRT()

    draw_sphere_marker(start_position, 0.01, [1, 0, 0, 1])
    position = getEndEffectorState(connecting_node)
    draw_sphere_marker(position, 0.01, [1, 0, 0, 1])

    green_path = DFS(T1, start_conf, connecting_node)
    blue_path = DFS(T2, goal_conf, connecting_node)[::-1]

    if green_path == None or blue_path == None:
        return None
    return green_path + blue_path


def check_collision(first, second):
    """
    Given two points, return whether there is a collision with 3D obstacles. 
    """
    while distance_euclid(first, second) > 0.001:
        first = progress(first, second, 0.001)
        if collision_fn(first):
            return True

    return False
    

def birrt_smoothing():
    ################################################################
    # TODO your code to implement the birrt algorithm with smoothing
    ################################################################
    """
    Build Path Configuration with two trees and try to connect them
    with a smoother and shorter path. 
    """
    T1, T2, connecting_node = build_biRRT()

    draw_sphere_marker(start_position, 0.01, [1, 0, 0, 1])
    position = getEndEffectorState(connecting_node)
    draw_sphere_marker(position, 0.01, [1, 0, 0, 1])

    green_path = DFS(T1, start_conf, connecting_node)
    blue_path = DFS(T2, goal_conf, connecting_node)[::-1]

    if green_path == None or blue_path == None:
        return None
    
    path = green_path + blue_path

    #Smoothing 
    for x in range(0, 100):
        first_point = randint(0, len(path)-1)
        upper = first_point + 5 
        lower = first_point - 5
        if lower < 0:
            lower = 0
        if upper > len(path)-1:
            upper = len(path)-1
        second_point = randint(lower, upper)
        while second_point == first_point:
            second_point = randint(lower, upper)
        if second_point < first_point:
            buf = first_point
            first_point = second_point
            second_point = buf
        
        #Check for collision between two nodes
        if not check_collision(path[first_point], path[second_point]):
            path = path[:first_point+1] + path[second_point:]
    
    #Interpolate the final path
    #Find a point between two points, interpolate x and y independently
    first_path = path[:-1]

    path = path[-2:]
    path = path[::-1]
    for i in range(100):
        j = 0 
        while True:
            try:
                first = path[j]
                second = path[j+1]
            except:
                break
            if distance_euclid(first,second) > 0.1:
                z = (first[2] + second[2])/2
                x = np.interp(z, (first[2], second[2]), (first[0], second[0]))
                y = np.interp(z, (first[2], second[2]), (first[1], second[1]))
                path.insert(j+1, (x,y,z))
                j = j + 2
            else:
                #didn't add a new node, just next one
                j = j + 1


        j = 0 
        while True:
            try:
                first = first_path[j]
                second = first_path[j+1]
            except:
                break
            if distance_euclid(first,second) > 0.1:
                z = (first[2] + second[2])/2
                x = np.interp(z, (first[2], second[2]), (first[0], second[0]))
                y = np.interp(z, (first[2], second[2]), (first[1], second[1]))
                first_path.insert(j+1, (x,y,z))
                j = j + 2
            else:
                #didn't add a new node, just next one
                j = j + 1      
    path = path[::-1]

    path = first_path + path

    return path

if __name__ == "__main__":
    args = get_args()

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.400,
        cameraYaw=58.000,
        cameraPitch=-42.200,
        cameraTargetPosition=(0.0, 0.0, 0.0),
    )

    # load objects
    plane = p.loadURDF("plane.urdf")
    ur5 = p.loadURDF(
        "assets/ur5/ur5.urdf", basePosition=[0, 0, 0.02], useFixedBase=True
    )
    obstacle1 = p.loadURDF(
        "assets/block.urdf", basePosition=[1 / 4, 0, 1 / 2], useFixedBase=True
    )
    obstacle2 = p.loadURDF(
        "assets/block.urdf", basePosition=[2 / 4, 0, 2 / 3], useFixedBase=True
    )
    obstacles = [plane, obstacle1, obstacle2]

    # start and goal
    start_conf = (-0.813358794499552, -0.37120422397572495, -0.754454729356351)
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    start_marker = draw_sphere_marker(
        position=start_position, radius=0.02, color=[1, 0, 0, 1]
    )
    goal_conf = (0.7527214782907734, -0.6521867735052328, -0.4949270744967443)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    goal_marker = draw_sphere_marker(
        position=goal_position, radius=0.02, color=[1, 0, 0, 1]
    )
    set_joint_positions(ur5, UR5_JOINT_INDICES, start_conf)

    # place holder to save the solution path
    path_conf = None

    # get the collision checking function
    from collision_utils import *

    collision_fn = get_collision_fn(
        ur5,
        UR5_JOINT_INDICES,
        obstacles=obstacles,
        attachments=[],
        self_collisions=True,
        disabled_collisions=set(),
    )

    if args.birrt:
        if args.smoothing:
            # using birrt with smoothing
            path_conf = birrt_smoothing()
        else:
            # using birrt without smoothing
            path_conf = birrt()
    else:
        # using rrt
        path_conf = rrt()

    if path_conf is None:
        # pause here
        raw_input("no collision-free path is found within the time budget, finish?")
    else:
        ###############################################
        # TODO your code to highlight the solution path
        ###############################################

        # Color the path
        prev_pos = start_position
        for i in range(len(path_conf)):
            # Plot the dots of the configuration in red
            position = getEndEffectorState(path_conf[i])
            draw_sphere_marker(position, 0.01, [1, 0, 0, 1])
            time.sleep(0.25)
            if i > 0:
                # create a red line with the previous node
                p.addUserDebugLine(prev_pos, position, [1, 0, 0], lineWidth=6)
                # update previous position to current
                prev_pos = position
        time.sleep(1)

        # execute the path, moves the robot arm given the path configuration in a infinite loop
        while True:
            for q in path_conf:
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                time.sleep(0.2)
