#!/usr/bin/python
from common import load_xy, load_diff, print_xy

def main(fgraph, fdiff):
    """
    Load xy_graph and pre-generated diff file,
    and output the new xy_graph with such diff
    """
    vert, arcs = load_xy(fgraph)
    gd = load_diff(fdiff)
    
    g = {}
    for arc in arcs:
        u, v, w = arc
        g[(u, v)] = w

    for u, v, w in gd:
        g[(u, v)] = w

    print_xy(vert, g)

if __name__ == "__main__":
    import sys
    xyfile = sys.argv[1]
    diffile = sys.argv[2]
    main(xyfile, diffile)
