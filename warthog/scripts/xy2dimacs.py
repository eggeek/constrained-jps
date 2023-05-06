#!/usr/bin/python
from common import load_xy, write_co, write_gr
import argparse

def main(xyfile: str, grfile: str, cofile: str):
    vert, arcs = load_xy(xyfile)
    write_co(vert, cofile)
    write_gr(vert, arcs, grfile)


if __name__ == "__main__":
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument("--i", nargs=1)
    parser.add_argument("--o", nargs=2, default=[None, None])
    namespace = parser.parse_args(sys.argv[1:])
    xyfile = str(namespace.i[0])
    grfile, cofile = map(str, namespace.o)
    if grfile is None:
        grfile = str(xyfile).strip(".xy") + ".gr"
        cofile = str(xyfile).strip(".xy") + ".co"
    main(xyfile, grfile, cofile)
