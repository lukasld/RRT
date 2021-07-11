# Rapidly Expanding Random Trees

<img src="https://i.ibb.co/k1LpS8F/rrt-single.png" alt="drawing" style="width:200px;"/>

This is an approach to implement RRT's from scratch, RRT's allow for multi-dimensional path-planning and object-collision-detection.<br>
The code is written entirely without `numpy`, it simply uses a library which depends on numpy to visualize the tree.
Currently the code supports two modes:

- RRT: includes a root-node the tree grows from there until it finds the target area and returns the found path
- bi-RRT: two root nodes are places and an overlap in the branch returns the found path

The algorithm is based and implemented from [Rapidly Expanding Random Trees original paper](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf)

### Todo ### 
- RRT* implementation
- make a library 
