#!/usr/bin/env python3

import matplotlib
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import networkx as nx
import math


from Euclidean_Elements import LineSegment
from RRT import RRT, RRTs, Node, Obstacle
from Solutionspace import SSpace

# TODO:
#   RRT*


if __name__ == '__main__':

    TOT_TRIES = 100
    TREE_ID_ONE = 0
    TREE_ID_TWO = 1

    root_node_one = Node((-80, -80), tree_id=TREE_ID_ONE)
    root_node_two = Node((80, 80), tree_id=TREE_ID_TWO)


    space_size = (100, 100)
    S = SSpace(space_size)

    target_node = Node((50, 50), target_area=3)

    #S.set_target(target_node)

    # both for other node and target
    S.set_target_delta(4)

    rrt1 = RRT(root_node_one, q_dist=4)
    rrt2 = RRT(root_node_two, q_dist=4)

    r1 = Obstacle([LineSegment((-80, 40), (-80, -20)),
                       LineSegment((-80, -20), (-60, -20)),
                       LineSegment((-60, -20), (-60, 20)),
                       LineSegment((-60, 20), (-40, 20)),
                       LineSegment((-40, 20), (-40, 30)),
                       LineSegment((-40, 30), (-60, 30)),
                       LineSegment((-60, 30), (-60, 40)),
                       LineSegment((-60, 40), (-80, 40))])


    r2 = Obstacle([LineSegment((-30, 40), (-30, -20)),
                       LineSegment((-30, -20), (-10, -20)),
                       LineSegment((-10, -20), (-10, 20)),
                       LineSegment((-10, 20), (10, 20)),
                       LineSegment((10, 20), (10, 30)),
                       LineSegment((10, 30), (-10, 30)),
                       LineSegment((-10, 30), (-10, 40)),
                       LineSegment((-10, 40), (-30, 40))])


    t = Obstacle([LineSegment((20, 30), (20, 20)),
                       LineSegment((20, 20), (30, 20)),
                       LineSegment((30, 20), (30, -20)),
                       LineSegment((30, -20), (50, -20)),
                       LineSegment((50, -20), (50, 20)),
                       LineSegment((50, 20), (60, 20)),
                       LineSegment((60, 20), (60, 30)),
                       LineSegment((60, 30), (50, 30)),
                       LineSegment((50, 30), (50, 40)),
                       LineSegment((50, 40), (30, 40)),
                       LineSegment((30, 40), (30, 30)),
                       LineSegment((30, 40), (30, 30)),
                       LineSegment((30, 30), (20, 30))])

    b_top = Obstacle([LineSegment((-120, 120), (-90, 90)),
                       LineSegment((-90, 90), (90, 90)),
                       LineSegment((90, 90), (120, 120)),
                       LineSegment((120, 120), (-120, 120))])


    b_bottom = Obstacle([LineSegment((-120, -120), (-90, -90)),
                       LineSegment((-90, -90), (90, -90)),
                       LineSegment((90, -90), (120, -120)),
                       LineSegment((120, -120), (-120, -120))])


    b_right = Obstacle([LineSegment((90, -90), (120, -120)),
                       LineSegment((120, -120), (120, 120)),
                       LineSegment((120, 120), (90, 90)),
                       LineSegment((90, 90), (90, -90))])


    b_left = Obstacle([LineSegment((-90, -90), (-120, -120)),
                       LineSegment((-120, -120), (-120, 120)),
                       LineSegment((-120, 120), (-90, 90)),
                       LineSegment((-90, 90), (-90, -90))])


    S.set_obstacles([r1, r2, t, b_top, b_bottom, b_right, b_left])

    # one or two trees
    S.set_rrt(rrt1)
    S.set_rrt(rrt2)

    # helper bi_rrt function
    def has_connection(target):
        if target:
            found = "Connection found w. Node: {} using Path: {}".format(target['tar_approx_node'], target['tar_approx_path'])
            print(found)
            return True
        return False


    # tot tries for either tree
    while len(S.rrt_list[0]) < TOT_TRIES:

        '''
        # w. target
        target = S.rrt_list[0].gen_node()
        if target:
            found = "Target found w. Node: {} using Path: {}".format(target['tar_approx_node'], target['tar_approx_path'])
            print(found)
            break
        '''

        # bi directional
        conn_a = S.rrt_list[0].gen_node(S.rrt_list[1])
        conn_b = S.rrt_list[1].gen_node(S.rrt_list[0])

        if any([has_connection(conn_a), has_connection(conn_b)]):
            break


    if len(S.rrt_list[0]) == TOT_TRIES:
        print("Target not found, try again")

    S.visualize()
