# Source code
```
├── maps
├── README.md
├── scenarios
└── warthog
```
- `maps`, `scenarios` store benchmark from `movingAI` and `IRON`
- `warthog` stores all source files

# Compile

Go to source file director: `cd warthog`
- To run benchmark: 
    - make fast: run in fast mode, without collecting statistic on scanning
    - make fastcnt: run in fast mode but collect statistic on scanning
- To debug: make dev
- To clean: make clean

Once compiled, run ./build/<dev|fast>/bin/warthog for a list of command line parameters. 
A simple case is the following:

./bin/warthog --scen <scen file> --map <map file> --alg <alg name>

Where map files are end with '.map', and scen file are end with '.scen'.

Algorithms are:
  - `--alg jps2`: JPS with block based scanning
  - `--alg jps2-prune2`: Constrained JPS

# Experiments

## Exp-1: Synthetic Maps

- `./exp1-synthetic.sh gen`: generate synthetic maps, store in `synthetic-data`:
```
├── diag-512-1-20p.map
├── diag-512-1.scen
├── ...
├── vary-size
│   ├── diag-1024.map
│   ├── diag-1024.scen
│   ├── diag-2048.map
│   ...

...
```

- `./exp1-synthetic.sh time`: run synthetic benchmark, measure performance, store in `synthetic-output`:
```
├── vary_br
│   ├── jps2
│   │   ├── 0/
│   │   ├── 1/
│   │   ...
│   └── jps2-prune2
│       ├── 0/
│       ├── 1/
│       ...
...
```

- `./exp1-synthetic.sh sub`: run synthetic benchmark, measure suboptimal node expansion/generation, store in `synthetic-output`:
```
├── vary_br
│   ├── jps2
│   │   ├── diag-512-0.75-0.1p.map.log
│   │   ├── diag-512-0.75-0.5p.map.log
│   │   ...
│   ├── jps2-prune2
...
...
```

## Exp-2: Ablation Study

For ablation study, checkout following branches:
  - `master`: CJPS
  - `jps-b`: backwards scanning on JPS
  - `jps-g`: diagonal caching on JPS
  - `cjps-b`: backwards scanning on CJPS
  - `cjps-g`: diagonal caching on CJPS

Then call script `exp2-ablation.sh`:

- `./exp2-ablation.sh time`: measure runtime
- `./exp2-ablation.sh sub`: measure suboptimal node generation/expansion

## Exp-3: Domain Maps

- `./domain-exp.sh gen_rand`: generate random obstacles on existing maps
- `./domain-exp.sh genjobs`: generate `domain-jobs.sh`. It includes all commands to run benchmarks with vary random obstacles and repeat 10 times, the order of these commands are shuffled to reduce caching behaviour.
- `./domain-jobs.sh`: run benchmarks to get performance data. Data are stored in `./domain-output`.
- `./domain-exp.sh gen_subq`: generate `subcnt_per_query.sh`. The generated scripts count the number of suboptimal node generation/expansion.
- `cat subcnt_per_query.sh | parallel`: it can be very slow as it run a Dijkstra for each query to compute a true distance table. Recommend to run in parallel.
