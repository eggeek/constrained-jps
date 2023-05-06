#!/bin/bash

maps=(
  ./synthetic-data/diag-512-0.75-0p.map
  ./synthetic-data/diag-512-0.75-0.1p.map
  ./synthetic-data/diag-512-0.75-0.5p.map
  ./synthetic-data/diag-512-0.75-1p.map
  ./synthetic-data/diag-512-0.75-1.5p.map
  ./synthetic-data/diag-512-0.75-10p.map
  ./synthetic-data/diag-512-0.75-20p.map
)

scens=(
  ./synthetic-data/diag-512-0.75.scen
  ./synthetic-data/diag-512-0.75.scen
  ./synthetic-data/diag-512-0.75.scen
  ./synthetic-data/diag-512-0.75.scen
  ./synthetic-data/diag-512-0.75.scen
  ./synthetic-data/diag-512-0.75.scen
  ./synthetic-data/diag-512-0.75.scen
  ./synthetic-data/diag-512-0.75.scen
  ./synthetic-data/diag-512-0.75.scen
)
algs=(jps2 jps2-prune2)
sml_outdir="./small_output/variants/"

function gensmljobs() {
  rep=10
  for (( r=0; r<rep; r++ )) {
    for (( i=0; i<${#maps[@]}; i++ )); do
      mpath=${maps[$i]}
      mapname=$(basename -- $mpath)
      spath=${scens[$i]}
      for alg in "${algs[@]}"; do
        outpath="${sml_outdir}/$alg/$r/"
        mkdir -p ${outpath}
        cmd="./build/fast/bin/warthog --scen ${spath} --map ${mpath} --alg $alg > ${outpath}/${mapname}.log"
        echo $cmd
      done
    done
  }
}

function runsmljobs() {
  gensmljobs | shuf > smljobs.sh
  chmod u+x smljobs.sh
  bash -x smljobs.sh
}

function small_suboptcnt() {
  make clean && make fastcnt -j
  for (( i=0; i<${#maps[@]}; i++ )); do
    mpath=${maps[$i]}
    mname=$(basename -- ${mpath})
    spath=${scens[$i]}
    out_dir="small_suboptcnt/variants/"
    mkdir -p ${out_dir}
    cmd="./build/fast/bin/experiment ${mpath} ${spath} subcnt > ${out_dir}/$mname.log"
    echo "$cmd"
    eval "$cmd"
  done
}

case "$1" in
  time) runsmljobs ;;
  sub) small_suboptcnt ;;
  *)
    echo "Usage: $0 {time|sub}"
    exit 1
esac
