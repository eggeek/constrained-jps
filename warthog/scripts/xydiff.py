#!/usr/bin/python
from common import load_xy, print_arcs

def main(f1: str, f2: str):
    _, arcs1 = load_xy(f1)
    _, arcs2 = load_xy(f2)
    
    g1 = {(u1, v1): w1 for u1, v1, w1 in arcs1}
    g2 = {(u2, v2): w2 for u2, v2, w2 in arcs2}

    diff = []
    for k, v in g1.items():
        if g2[k] != v:
            diff.append((k[0], k[1], g2[k]))
    print_arcs(diff)


if __name__ == "__main__":
    import sys
    xyfile1 = sys.argv[1]
    xyfile2 = sys.argv[2]
    main(xyfile1, xyfile2)
