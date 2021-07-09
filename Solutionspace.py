import matplotlib
from matplotlib import pyplot as plt
from matplotlib.pyplot import figure as fig
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import networkx as nx
import math



class SSpace:
    """ Wrapper Class
    """
    obstacles = []
    rrt_list = []

    def __init__(self, xy_ext):
        SSpace.xy_ext = xy_ext
        SSpace.target = None
        SSpace.target_delta = None # is used both for target or other tree node
        matplotlib.rcParams['figure.figsize'] = [xy_ext[0], xy_ext[1]] # for square canvas

    def set_obstacles(self, new_obstcls: list):
        """ adds a new obstacle to the obstacle list
        """
        SSpace.obstacles = new_obstcls

    def set_target(self, target_node):
        """ adds a new target
        """
        assert len(self.rrt_list) > 0, 'Set the tree first'
        assert len(self.rrt_list) <= 1, 'Target not allowed, with more than one Tree'
        SSpace.target = target_node

    def set_target_delta(self, target_delta):
        """ adds a new target
        """
        SSpace.target_delta = target_delta

    def set_rrt(self, rrt):
        """ adds the rrt
        """
        assert SSpace.target and len(self.rrt_list) <= 1 or not SSpace.target, 'Target not allowed, with more than one Tree'
        SSpace.rrt_list.append(rrt)

    def visualize(self):
        """ visualizes the solutionspace including target
        """

        # controls the final size
        plt.figure(figsize=(10, 10))
        plt.axis('equal')

        plt.xlim(-SSpace.xy_ext[0], SSpace.xy_ext[0])
        plt.xlim(-SSpace.xy_ext[1], SSpace.xy_ext[1])
        plt.autoscale(False)

        g1 = nx.Graph()
        g2 = nx.Graph()

        f_c = (g1, g2)

        if SSpace.rrt_list:
            for i, rrt in enumerate(SSpace.rrt_list):
                node_dict, edge_lst = rrt.gen_node_edge(rrt.root_node)
                f_c[i].add_nodes_from(node_dict.keys())
                nx.set_node_attributes(f_c[i], node_dict, 'pos')
                f_c[i].add_edges_from(edge_lst)


        if SSpace.target:
            assert SSpace.target.target_area is not None, "the target node must have a target_area"
            t_x, t_y = SSpace.target.x, SSpace.target.y
            target = plt.Circle((t_x, t_y), SSpace.target.target_area, color='r')
            plt.gcf().gca().add_artist(target)

        if SSpace.obstacles:
            for obstacle in SSpace.obstacles:
                patches = []
                polygon = Polygon(obstacle.verts, True)
                patches.append(polygon)
                p = PatchCollection(patches)
                plt.gcf().gca().add_artist(p)

        # f_c = nx.compose(f_c[0], f_c[1])

        nx.draw_networkx(f_c[0],
                         pos=nx.get_node_attributes(f_c[0], 'pos'),
                         node_color='white',
                         node_size=50,
                         edge_color='red',
                         font_color='red',
                         font_size=5)

        nx.draw_networkx(f_c[1],
                         pos=nx.get_node_attributes(f_c[1], 'pos'),
                         node_color='white',
                         node_size=50,
                         edge_color='orange',
                         font_color='orange',
                         font_size=5)

        plt.show()

