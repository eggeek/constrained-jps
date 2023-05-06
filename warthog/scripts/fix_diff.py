#!/usr/bin/python
from common import load_g, print_g

def main(fgraph, fdiff):
    """
    The original diff file is buggy - some edges have smaller weight than in the original map,
    this causes cpd-search not optimal;
    This script is to fix this issue by ignoring those edges.
    """
    g1 = load_g(fgraph)
    g2 = load_g(fdiff)
    gd = {}

    for k in g2.keys():
        if g1.get(k) is None or g1[k] >= g2[k]:
            continue
        else:
            gd[k] = g2[k]

    print_g(gd)

if __name__ == "__main__":
    import sys
    f1 = sys.argv[1]
    f2 = sys.argv[2]
    main(f1, f2)
