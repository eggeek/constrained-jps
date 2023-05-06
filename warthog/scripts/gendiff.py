#!/usr/bin/python
from common import load_xy, print_arcs
import random

ratio: float
rl: float
ru: float

def random_perturbation(arcs):
    num = int(len(arcs) * ratio)
    random.shuffle(arcs)

    perturbed = []
    for i in range(num):
        u, v, w = arcs[i]
        r = random.uniform(rl, ru)
        perturbed.append((u, v, int(w * r)))
    return perturbed


def main(argv: [str]):
    global ratio, rl, ru
    ratio, rl, ru = 0.1, 1.0, 1.5
    xyfile: str = argv[1]
    if len(argv) >= 3:
        ratio = float(argv[2])
    if len(argv) >= 5:
        rl, ru = float(argv[3]), float(argv[4])
    _, arcs = load_xy(xyfile)
    perturbed = random_perturbation(arcs)
    print_arcs(perturbed)


if __name__ == "__main__":
    import sys
    main(sys.argv)
