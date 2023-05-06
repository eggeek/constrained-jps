#!/usr/bin/env python
import sys
import random
import os
from math import sqrt

grids = []
start = []
target = []
obstacle_marks = "SWT@O"

def fill(x0, y0, x1, y1, c):
    for x in range(x0, x1+1):
        for y in range(y0, y1+1):
            grids[y][x] = c

def print_grids():
    print ("type octile")
    print ("height %d" % l)
    print ("width %d" % l)
    print ("map")
    for i in range(l):
        print (''.join(grids[i]))

def gen_empty(l):
    global grids
    grids = [['.'] * l for _ in range(l)]
    fill(l-2, 0, l-2, l-1, 'T')
    print_grids()

def gen_maxscan(l):
    global grids
    grids = [['.'] * l for _ in range(l)]

    # add obstacles on border
    fill(0, 0, l-1, 0, 'T')
    fill(0, 0, 0, l-1, 'T')
    fill(l-1, 0, l-1, l-1, 'T')
    fill(0, l-1, l-1, l-1, 'T')

    #  fill(1, 1, 1, l-1, 'T')
    #  fill(1, 1, 1, l//4, '.')

    for x in range(1, l // 8, 2):
        for y in range(1, l-1):
            if x % 4 == 1:
                grids[y][x] = 'T' if y % 2 == 0 else '.'
            elif x % 4 == 2:
                grids[y][x] = '.'
            elif x % 4 == 3:
                grids[y][x] = '.' if y % 2 == 0 else 'T'

    for y in range(2, l-1):
        if y % 4:
            fill(l // 8 + 1, y, l // 2, y, 'T')

    fill(l-3, 0, l-3, l-1, 'T')
    grids[1][1] = '.'
    grids[l-1][l-1] = '.'

    print_grids()


def gen_square_grid(l, r):
    global grids
    grids = [['.'] * l for _ in range(l)]

    # add obstacles on border
    fill(0, 0, l-1, 0, 'T')
    fill(0, 0, 0, l-1, 'T')
    fill(l-1, 0, l-1, l-1, 'T')
    fill(0, l-1, l-1, l-1, 'T')

    # mid line obstacles
    for x in range(l // 8, l - l // 8):
        grids[l // 2][x] = 'T'

    # mid random obstacles
    for y in range(l*3 // 8, l*5 // 8):
        for x in range(1, l):
            if random.random() < r:
                grids[y][x] = 'T'

    print_grids()

def gen_square_query(l, num=100):
    with open(mapfile, 'r') as f:
        grids = f.readlines()[4:]
    h = len(grids)
    w = len(grids[0])
    global start, target
    start, target = [], []

    while len(start) < num:
        x = random.randint(1, w-1)
        y = random.randint(1, h // 8)
        if (grids[y][x] == '.'):
            start.append((x, y))

    while len(target) < num:
        x = random.randint(1, w-1)
        y = h - random.randint(1, h // 8)
        if (grids[y][x] == '.'):
            target.append((x, y))


    print (mapfile)
    print (min(len(start), len(target)))
    for i in range(min(len(start), len(target))):
        print (start[i][0], start[i][1], target[i][0], target[i][1])


def gen_diag_grid(l, r, digR=0.75):
    """
    l * l map

    x = 0, x = l-1 and y = 0, y = l-1 are obstacle

    random obstacle in the middle

    start: above obstacles
    target: below obstacles
    """
    global grids
    grids = [['.'] * l for _ in range(l)]

    h = int(float(l) * r)

    # add obstacles on border
    fill(0, 0, l-1, 0, 'T')
    fill(0, 0, 0, l-1, 'T')
    fill(l-1, 0, l-1, l-1, 'T')
    fill(0, l-1, l-1, l-1, 'T')

    # blockage length is diagonal length * diagR, and the blockage is at the middle
    # diagR control the ratio of empty segment 
    # where diag in {0, 0.25, 0.5, 0.75, 1}
    # diagR = 1: fully blocked, diagR = 0: no blockage
    if digR == 1:
        for x in range(l):
            y = l-1-x
            grids[y][x] = 'T'
    else:
        bl = int(l * digR)
        sx = (l - int(l*digR)) // 2
        sy = l - (l - int(l*digR)) // 2
        for i in range(bl):
            y = sy - i
            x = sx + i
            grids[y][x] = 'T'

    for d in range(1, l // 8):
        for x in range(l - d):
            y = l - d - x
            if random.random() < r:
                grids[y][x] = 'T'
            #  if not random.randint(0, 31):
            #      grids[y][x] = 'T'

    for d in range(1, l // 8):
        for x in range(d, l):
            y = l-1 + d - x
            if random.random() < r:
                grids[y][x] = 'T'
            #  if not random.randint(0, 31):
            #      grids[y][x] = 'T'

    print_grids()


def gen_diag_scen(mapfile, num):
    with open(mapfile, 'r') as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
    h = len(grids)
    w = len(grids[0])
    global start, target
    start, target = [], []

    while len(start) < num:
        x = random.randint(1, w // 8)
        y = random.randint(1, h // 8)
        if (grids[y][x] == '.'):
            start.append((x, y))

    while len(target) < num:
        x = w - random.randint(1, w // 8)
        y = h - random.randint(1, h // 8)
        if (grids[y][x] == '.'):
            target.append((x, y))

    print("version 1")
    for i in range(len(start)):
        print("0 %s %d %d %d %d %d %d 0" % (mapfile, h, w, start[i][0], start[i][1], target[i][0], target[i][1]))

    #  print (mapfile)
    #  print (min(len(start), len(target)))
    #  for i in range(min(len(start), len(target))):
    #      print (start[i][0], start[i][1], target[i][0], target[i][1])

def gen_query(mapfile, num):
    with open(mapfile, 'r') as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
    h = len(grids)
    w = len(grids[0])
    global start, target
    start, target = [], []

    while len(start) < num:
        x = random.randint(0, w-1)
        y = random.randint(0, h-1)
        if (grids[y][x] == '.'):
            start.append((x, y))

    while len(target) < num:
        x = random.randint(0, w-1)
        y = random.randint(0, h-1)
        if (grids[y][x] == '.'):
            target.append((x, y))

    print (mapfile)
    print (min(len(start), len(target)))
    for i in range(min(len(start), len(target))):
        print (start[i][0], start[i][1], target[i][0], target[i][1])

def query2scen(qfile):
    with open(qfile, 'r') as f:
        raw = f.readlines()
    mapfile = raw[0].strip()
    with open(mapfile, 'r') as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
    h = len(grids)
    w = len(grids[0])
    global start, target
    start, target = [], []
    for line in raw[2:]:
        sx, sy, tx, ty = map(int, line.split(' '))
        start.append((sx, sy))
        target.append((tx, ty))
    print("version 1")
    for i in range(len(start)):
        print("0 %s %d %d %d %d %d %d 0" % (mapfile, h, w, start[i][0], start[i][1], target[i][0], target[i][1]))


def gen_scen(mapfile, num):
    with open(mapfile, 'r') as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
        h = len(grids)
        w = len(grids[0])
        global start, target
        start, target = [], []

        while len(start) < num:
            x = random.randint(0, w-1)
            y = random.randint(0, h-1)
            if (grids[y][x] == '.'):
                start.append((x, y))

        while len(target) < num:
            x = random.randint(0, w-1)
            y = random.randint(0, h-1)
            if (grids[y][x] == '.'):
                target.append((x, y))

        print ("version 1")
        for i in range(min(len(start), len(target))):
            print ("0 %s %d %d %d %d %d %d 0" % (mapfile, h, w, 
                start[i][0], start[i][1], target[i][0], target[i][1]))

def count_traversable(mapfile):
    with open(mapfile, "r") as f:
        rows = [i.strip() for i in f.readlines()[4:]]
    mapw = len(rows[0])
    maph = len(rows)
    cnt = 0
    for y in range(maph):
        for x in range(mapw):
            if (rows[y][x] not in obstacle_marks):
                cnt += 1
    return mapw, maph, cnt

def domain_traversable():
    domains = ["iron", "starcraft", "bgmaps", "dao", "street", "maze512", "rooms", "random10"]
    dir = "./maps"
    header = "domain,map,mapw,maph,traversable"
    print (header)
    for domain in domains:
        for map in os.listdir("{}/{}/".format(dir, domain)):
            mpath = "{}/{}/{}".format(dir, domain, map)
            mapw, maph, cnt = count_traversable(mpath)
            row = "{},{},{},{},{}".format(domain, map, mapw, maph, cnt)
            print (row)

def randomlize(mapfile, ratio):
    with open(mapfile, "r") as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
    h = len(grids)
    w = len(grids[0])

    cnt = 0
    for i in range(h):
        for j in range(w):
            if grids[i][j] == '.':
                cnt += 1

    num = cnt * ratio

    while (num > 0):
        row = random.randint(0, h-1)
        col = random.randint(0, w-1)
        if (grids[row][col] == '.'):
            grids[row][col] = '@'
            num -= 1

    print ("type octile\nheight %d\nwidth %d\nmap" % (h, w))
    for i in range(h):
        print(''.join(grids[i]))

def valid(mapfile, scenfile):
    with open(mapfile, "r") as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
    st = set()
    with open(scenfile, "r") as f:
        rows = f.readlines()[1:]
        for row in rows:
            if not row.strip():
                continue
            sx, sy, tx, ty = map(int, row.split()[4:-1])
            st.add((sx, sy))
            st.add((tx, ty))
    for x, y in st:
        if grids[y][x] != ".":
            return False
    return True

def valid_all():
    domains = ["bgmaps", "iron", "random10", "starcraft", "street", "dao",
            "maze512", "rooms"]
    datasets = [
        "./data/maps-randomlized-0.1p",
        "./data/maps-randomlized-0.5p",
        "./data/maps-randomlized-1p",
        "./data/maps-randomlized-1.5p"
    ]
    for dataset in datasets:
        for domain in domains:
            for mfile in os.listdir(dataset + "/" + domain):
                mpath = dataset + "/" + domain + "/" + mfile
                spath = "./scenarios/movingai/" + domain + "/" + mfile + ".scen"
                #  print (mpath, spath)
                if (not valid(mpath, spath)):
                    print (mpath, spath)

def randomlize_respect_scen(mapfile, scenfile, ratio=0.015):
    random.seed(0)
    with open(mapfile, "r") as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
    st = set()
    with open(scenfile, "r") as f:
        rows = f.readlines()[1:]
        for row in rows:
            if not row.strip():
                continue
            sx, sy, tx, ty = map(int, row.split()[4:-1])
            st.add((sx, sy))
            st.add((tx, ty))
    h = len(grids)
    w = len(grids[0])

    nodes = []
    cnt = 0
    for y in range(h):
        for x in range(w):
            if grids[y][x] == '.':
                cnt += 1
            if grids[y][x] == '.' and ((x, y) not in st):
                nodes.append((x, y))
    random.shuffle(nodes)
    num = min(int(cnt * ratio), len(nodes))
    for x, y in nodes[:num]:
        assert(grids[y][x] == '.')
        assert((x, y) not in st)
        grids[y][x] = '@'

    print ("type octile\nheight %d\nwidth %d\nmap" % (h, w))
    for i in range(h):
        print(''.join(grids[i]))
    print ("#traversable: ", cnt, " #added obstacles: ", num, 
            " ratio: ", float(num) / float(cnt), file=sys.stderr)

def scale_up(mapfile: str, r: int):
    with open(mapfile, "r") as f:
        grids = [list(i.strip()) for i in f.readlines()[4:]]
    h = len(grids)
    w = len(grids[0])

    newh = h * r
    neww = w * r
    newg = [['.']*neww for i in range(newh)]

    assert(neww * newh == h*w*r**2)

    for y in range(newh):
        for x in range(neww):
            newg[y][x] = grids[y // r][x // r]

    print ("type octile\nheight %d\nwidth %d\nmap" % (newh, neww))
    for i in range(newh):
        print(''.join(newg[i]))

def scale_up_scen(sfile: str, r: int):
    header = ["bucket_id", "map", "h", "w", "sx", "sy", "tx", "ty", "ref_dist"]
    import pandas as pd
    df: pd.DataFrame = pd.read_csv(sfile, sep=' ', header=0, names=header)
    df['bucket_id'] *= r
    df['h'] *= r
    df['w'] *= r
    df['sx'] *= r
    df['sy'] *= r
    df['tx'] *= r
    df['ty'] *= r
    df['ref_dist'] *= r
    print('version 1')
    print(df.to_csv(header=False, sep=' ', index=False))

def grid2csv(mfile):
    with open(mfile, "r") as f:
        rows = [i.strip() for i in f.readlines()[4:]]
    header = "map\tmapw\tmaph\tx\ty\ttraversable"
    print(header)
    mapw = len(rows[0])
    maph = len(rows)
    for y in range(maph):
        for x in range(mapw):
            b = rows[y][x] in obstacle_marks
            print("%s\t%d\t%d\t%d\t%d\t%d" % (mfile, mapw, maph, x, y, b))


if __name__ == "__main__":
    """
    ./gen map <l> <r>
    ./gen query <mapfile> <num>
    """
    if (sys.argv[1] == "diag-map"):
        l = int(sys.argv[2])
        r = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.01
        dr = float(sys.argv[4])
        gen_diag_grid(l, r, digR=dr)
    elif (sys.argv[1] == 'square-map'):
        l = int(sys.argv[2])
        r = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.01
        gen_square_grid(l, r)
    elif (sys.argv[1] == "maxscan"):
        l = int(sys.argv[2])
        gen_maxscan(l)
    elif (sys.argv[1] == "diag-scen"):
        mapfile = sys.argv[2]
        num = int(sys.argv[3]) if len(sys.argv) >= 4 else 100
        gen_diag_scen(mapfile, num)
    elif (sys.argv[1] == 'square-query'):
        mapfile = sys.argv[2]
        num = int(sys.argv[3]) if len(sys.argv) >= 4 else 100
        gen_square_query(mapfile, num)
    elif (sys.argv[1] == "query"):
        mapfile = sys.argv[2]
        num = int(sys.argv[3]) if len(sys.argv) >= 4 else 100
        gen_query(mapfile, num)
    elif (sys.argv[1] == "scen"):
        mapfile = sys.argv[2]
        num = int(sys.argv[3]) if len(sys.argv) >= 4 else 100
        gen_scen(mapfile, num)
    elif (sys.argv[1] == 'empty'):
        l = int(sys.argv[2])
        gen_empty(l)
    elif (sys.argv[1] == "rand"):
        mapfile = sys.argv[2]
        ratio = float(sys.argv[3])
        randomlize(mapfile, ratio)
    elif (sys.argv[1] == "rand_scen"):
        mapfile = sys.argv[2]
        scenfile = sys.argv[3]
        ratio = 0.01
        if (len(sys.argv) >= 5):
            ratio = float(sys.argv[4])
        randomlize_respect_scen(mapfile, scenfile, ratio=ratio)
    elif (sys.argv[1] == "scale"):
        mapfile = sys.argv[2]
        f = int(sys.argv[3])
        scale_up(mapfile, f)
    elif (sys.argv[1] == "scale-scen"):
        sfile = sys.argv[2]
        f = int(sys.argv[3])
        scale_up_scen(sfile, f)
    elif (sys.argv[1] == "query2scen"):
        qfile = sys.argv[2]
        query2scen(qfile)
    elif (sys.argv[1] == "grid2csv"):
        mfile = sys.argv[2]
        grid2csv(mfile)
    elif (sys.argv[1] == "cnt"):
        domain_traversable()
