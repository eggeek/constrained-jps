#!/usr/bin/python
def load_g(fname):
    d = {}
    with open(fname, "r") as f:
        for line in f.readlines():
            if line[0] == 'e':
                _, u, v, w = line.strip().split(' ')
                key = (int(u), int(v))
                w = int(w)
                if d.get(key) is not None:
                    d[key] = min(d[key], w)
                else:
                    d[key] = w
    return d


def print_g(g):
    print (len(g.keys()))
    for k in sorted(g.keys()):
        print (k[0], k[1], g[k])


def load_xy(fname):
    vert = []
    arcs = []
    with open(fname, "r") as f:
        for line in f.readlines():
            if line[0] == 'n':
                _, vnum, _, enum = line.strip().split()
                vnum = int(vnum)
                enum = int(enum)
            elif line[0] == 'v':
                _, idx, x, y = line.strip().split()
                vert.append((int(idx), int(x), int(y)))
            elif line[0] == 'e':
                _, u, v, w = line.strip().split()
                arcs.append((int(u), int(v), int(w)))
    return vert, arcs


def load_diff(fname):
    arcs = []
    with open(fname, "r") as f:
        lines = f.readlines()
        anum = int(lines[0].strip())
        for i in range(anum):
            u, v, w = map(int, lines[i+1].strip().split())
            arcs.append((u, v, w))
    return arcs

def write_gr(vert, arcs, fname: str):
    with open(fname, "w") as f:
        f.write("p sp %d %d\n" % (len(vert), len(arcs)))
        for u, v, w in arcs:
            f.write("a %d %d %d\n" % (u+1, v+1, w))
    
def write_co(vert, fname: str):
    with open(fname, "w") as f:
        f.write("p aux sp co %d\n" % len(vert))
        for i, x, y in vert:
            f.write("v %d %d %d\n" % (i+1, x, y))

def print_xy(vert, g):
    print ("nodes", len(vert), "edges", len(g.keys()))
    for v in vert:
        print ("v", v[0], v[1], v[2])
    for e, w in g.items():
        print ("e", e[0], e[1], w)


def print_arcs(arcs):
    print (len(arcs))
    for u, v, w in arcs:
        print(u, v, int(w))
