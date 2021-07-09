import math
import random
import concurrent.futures
from Solutionspace import SSpace
from Euclidean_Elements import Vector, LineSegment



class Obstacle(LineSegment):
    """ Obstacle defined as list of vectors
        vectors need to be in order
        :pt_list a list of LineSegments
    """

    def __init__(self, ln_list: list = []):
        if not self._is_consistent:
            raise ValueError ('endpoints and startpoints of consecutive lines must be matching')
        self.ln_list = ln_list

    def _is_consistent(self):
        """ tests whether start/endpoints of lines are matching
        """
        consistency = []
        for i in range(len(self.ln_list)):
            consistency.append(self.ln_list[i] == (self.ln_list[(i+1) % len(self.ln_list)]))
        return all(consistency)

    def does_intersect(self, x_ln: LineSegment):
        """ checks if a line intersects w. the polyline
        """
        for ln in self.ln_list:
            if ln.intersect(x_ln):
                return True
        return False

    @property
    def verts(self):
        verts = []
        for ln in self.ln_list:
            p0, p1 = ln.get_endpoints()
            verts.extend((p0, p1))
        #TODO: get rid of duplicates wile maintaining draw order
        return verts

    @classmethod
    def from_vec(cls, vec_list: list):
        """ construct from Vector
        """
        cls.verts = vec_list
        return cls()

    def __repr__(self):
        return 'Obstacle({})'.format(self.ln_list)



class Node(Vector):
    """ Node in the solution space in R^2
    """

    # dict of dict containing, nodes for each tree
    tot_nodes_dict = {}

    def __init__(self, xy: tuple, tree_id=None,
                                  parentNode=None,
                                  is_qrand=False,
                                  target_area=None):
        super().__init__(xy)

        if not is_qrand:
            self.parentNode = parentNode
            self.connections = [] #should be named children

            if tree_id not in Node.tot_nodes_dict:
                Node.tot_nodes_dict[tree_id] = 0
            else:
                Node.tot_nodes_dict[tree_id] += 1
            self.nr = self.tot_nodes_dict[tree_id]
        if target_area:
            self.target_area = target_area

    @property
    def edge(self) -> LineSegment:
        """ returns the edge as a LineSegment
        """
        return LineSegment((self.parentNode.x, self.parentNode.y), (self.x, self.y))

    @classmethod
    def vec(cls, vec: Vector, parentNode, tree_id):
        """ constructor from Vector
        """
        return cls((vec.x, vec.y), parentNode, tree_id)

    def __repr__(self):
        return "Node('{}', '{}', '{}')".format(self.nr, (self.x, self.y), len(self.connections))



class Tree:

    id = 0
    id_map = {1:'a', 2:'b'}

    def __init__(self, root_node):
        self.root_node = root_node
        self.node_pool = {0: (root_node, 0)}
        Tree.id += 1
        self.id = Tree.id

    def gen_node_edge(self, node: Node, node_dct={}, edge_lst=[]) -> tuple:
        """ traverses tree using bfs, returns a node_dct and a edge_dct
        """
        node_nr = str(node.nr)+ Tree.id_map[self.id]
        if node:
            # I chose dfs
            node_dct[node_nr] = (node.x, node.y)
            for new_node in node.connections:
                new_node_nr = str(new_node.nr) + Tree.id_map[self.id]
                # adding the edges
                edge_lst.append((node_nr, new_node_nr))
                # adding the nodes
                self.gen_node_edge(new_node)

            return (node_dct, edge_lst)



class RRT(Tree, SSpace):

    """ Typical RRT
    """
    sspace = None

    def __init__(self, root_node, q_dist):
        Tree.__init__(self, root_node)
        RRT.sspace = SSpace
        self.q_dist = q_dist

    def gen_q_rand(self):
        """ random Node between xy_ext includes negative space
        """
        xy_pos = (random.uniform(-RRT.sspace.xy_ext[0], RRT.sspace.xy_ext[0]),
                  random.uniform(-RRT.xy_ext[1], RRT.xy_ext[1]))
        return Node(xy_pos, is_qrand=True)

    def get_node_pool(self, id) -> Node:
        """ gets Node by Id
        """
        return self.node_pool[id]

    def add_node_pool(self, node: Node):
        """ add node to pool
        """
        node, id = node, node.nr
        self.node_pool[id] = (node, 0)

    def _find_closest_node(self, q_target: Node) -> Node:
        """ we loop through all of the nodes and find the closest
        """
        for node, _ in self.node_pool.values():
            new_vec = node - q_target
            self.node_pool[node.nr] = (node, new_vec.magnitude)
        return min(self.node_pool.values(), key=lambda x: x[1])[0]

    def gen_node(self, other=None):
        """ generates the next Node
        """
        q_rand = self.gen_q_rand()
        q_closest = self._find_closest_node(q_rand)
        vec_rand = q_closest - q_rand
        if vec_rand.magnitude >= self.q_dist:
            vec_rand = vec_rand.v_hat.scalar_mult(self.q_dist)
            new_node = Node.vec((q_closest + vec_rand), self.id, q_closest)
            # obstacle intersection
            ln_intersect = []
            for obst in RRT.sspace.obstacles:
                ln_intersect.append(obst.does_intersect(new_node.edge))
            # line intersection
            if not any(ln_intersect):
                q_closest.connections.append(new_node)
                self.add_node_pool(new_node)
                poss_path = None
                # if we have a target
                if RRT.sspace.target:
                    poss_path = self._check_target_prox(new_node)
                # if we have anothe tree
                elif other:
                    poss_path = self._check_other_prox(new_node, other)
                if poss_path: return poss_path
            else:
                Node.tot_nodes_dict[self.id] -= 1
        return False

    def _check_target_prox(self, node):
        """ checks for target proximity if it is within radius of target_delta
        """
        target_vec = node - RRT.sspace.target
        if target_vec.magnitude <= RRT.sspace.target_delta:
            path = self.gen_path_origin_node(node)
            return {'tar_approx_node' : node, 'tar_approx_path' : path}
        return False


    def _check_other_prox(self, node, other):
        """ checks if node is within radius of target_delta of other tree
        """
        poss_nodes = []
        for node_o, _ in other.node_pool.values():
            poss_nodes.append((node, (node - node_o).magnitude))
        cl_n, cl_n_magnitude = sorted(poss_nodes, key=lambda x: x[1])[0]
        if cl_n_magnitude <= RRT.sspace.target_delta:
            self_path = self.gen_path_origin_node(node)
            other_path = other.gen_path_origin_node(cl_n)
            return {'tar_approx_node' : node, 'tar_approx_path' : self_path+other_path}
        return False


    #TODO: Multiprocessing for collision detection
    '''
    def _check_poss_obst_intersections(self, new_node):
        """ Multiprocessing of Object intersections
        """
        all_res = []
        with concurrent.futures.ProcessPoolExecutor(max_workers=4) as executor:
            results = [executor.submit(obst.does_intersect, new_node.edge) for obst in RRT.sspace.obstacles]
            for f in concurrent.futures.as_completed(results):
                all_res.append(f.result())
        return not any(all_res)
    '''

    def gen_path_origin_node(self, node: Node, path=[]):
        """ the path to a node from origin
        """
        path.append(node)
        if node == self.root_node or node is None:
            return path[::-1]
        return self.gen_path_origin_node(node.parentNode)


    def __len__(self):
        return len(self.node_pool.keys())



class RRTs(RRT):

    """ RRT Star
    """
    def __init__(self, root_node, q_dist, q_n_multiplier):
        super().__init__(root_node, q_dist)
        self.q_neigh_dist = self.q_dist * q_n_multiplier

        # 1. loop through the Neighbors, find the one which would make the path to root shorter, from q_nearest
        #       # each node needs to have the value of distance to root
        #       # then we can check distance
        # 2. loop through the negihbors with the new endnode and find connections which would shorten paths

    def gen_node(self):
        """ different gen node from RRT
        """

        q_rand = self.gen_q_rand()
        q_closest = self._find_closest_node(q_rand)
        vec_rand = q_closest - q_rand
        vec_rand_scaled = vec_rand.v_hat.scalar_mult(self.q_dist)
        new_node = Node.vec((q_closest + vec_rand_scaled), q_closest)

        '''
        ln_intersect = []
        # TODO: Multiprocessing
        for obst in RRT.sspace.obstacles:
            ln_intersect.append(obst.does_intersect(new_node.edge))
        '''

        does_not_intersect = self._check_poss_obst_intersections(new_node)

        # here we check if we do not intersect with any of the objects
        if does_not_intersect:
            q_closest.connections.append(new_node)
            self.add_node_pool(new_node)
            if RRT.sspace.target:
                poss_path = self._check_target_prox(new_node)
                if poss_path:
                    return poss_path
        else:
            Node.tot_nodes -= 1
        return False

    def is_in_neighborhood(self, center_n):
        """ returns a list of neighboring nodes
        """
        for node, _ in self.node_pool.values():
            new_vec = node - q_target
            self.node_pool[node.nr] = (node, new_vec.magnitude)
        return min(self.node_pool.values(), key=lambda x: x[1])[0]

        #
        # self.q_neigh_dist
        # self._find_closest_node(q_target)
        #
        return
