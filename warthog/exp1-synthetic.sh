#!/bin/bash

# ratio of random obstacles
rs=(0.1 0.5 1 1.5 10 20)
# size of map l by l
lens=(256 512 1024 2048)
# control the size of blocked segment
# 0: no blockage
# 0.25: 25% of diagonal blockage in the middle, ...
# 1: fully blocked
drs=(0 0.25 0.5 0.75 1)
dir="./synthetic-data"
sml_outdir="./synthetic-output"

function gen_maps() {
  mkdir -p ${dir}
  mkdir -p ${sml_outdir}
  for l in "${lens[@]}"; do
    for dr in "${drs[@]}"; do
      r=0
      map="${dir}/diag-${l}-${dr}-${r}p.map"
      scen="${dir}/diag-${l}-${dr}.scen"

      cmd="./gen.py diag-map ${l} ${r} ${dr} > ${map}"
      echo "generating map: ${cmd}"
      eval $cmd

      cmd="./gen.py diag-scen ${map} > ${scen}"
      echo "generating scen: ${cmd}"
      eval $cmd

      for r in "${rs[@]}"; do
        rp=$(python -c "print(${r} / 100.0)")
        map0="${dir}/diag-${l}-${dr}-0p.map"
        map="${dir}/diag-${l}-${dr}-${r}p.map"
        scen="${dir}/diag-${l}-${dr}.scen"

        cmd="./gen.py rand_scen ${map0} ${scen} ${rp} > ${map}"
        echo "generating map: ${cmd}"
        eval $cmd
      done
    done
  done

  # gen vary resolution maps ans scens
  mkdir -p "${dir}/vary-size"
  for s in $(seq 0 3); do
    map0="${dir}/diag-256-0.75-0.1p.map"
    scen0="${dir}/diag-256-0.75.scen"
    v=$(python -c "print (2**${s})")
    l=$(python -c "print (${v} * 256)")
    map="${dir}/vary-size/diag-${l}.map"
    scen="${dir}/vary-size/diag-${l}.scen"
    cmd="./gen.py scale ${map0} ${v} > ${map}"
    echo "${cmd}"
    eval $cmd

    cmd="./gen.py scale-scen ${scen0} ${v} > ${scen}"
    echo "${cmd}"
    eval $cmd
  done
}

function _gen_jobs_time() {
  for l in "${lens[@]}"; do
    for dr in "${drs[@]}"; do
      for r in "${rs[@]}"; do
        for alg in "${algs[@]}"; do
          for (( i=0; i<rep; i++ )) {
            exe="./build/fast/bin/warthog"
            scen="${dir}/diag-${l}-${dr}.scen"
            map="${dir}/diag-${l}-${dr}-${r}p.map"
            mapname=$(basename -- $map)
            outpath="${sml_outdir}/${alg}/$i/"
            mkdir -p ${outpath}
            cmd="${exe} --scen ${scen} --map ${map} --alg ${alg} > ${outpath}/${mapname}.log"
            echo $cmd
          }
        done
      done
    done
  done
}

function _gen_jbos_subcnt() {
  for l in "${lens[@]}"; do
    for dr in "${drs[@]}"; do
      for r in "${rs[@]}"; do
        for alg in "${algs[@]}"; do
          exe="./build/fast/bin/experiment"
          scen="${dir}/diag-${l}-${dr}.scen"
          map="${dir}/diag-${l}-${dr}-${r}p.map"
          mapname=$(basename -- $map)
          outpath="${sml_outdir}/${alg}/"
          mkdir -p ${outpath}
          cmd="${exe} ${map} ${scen} subcnt > ${outpath}/${mapname}.log"
          echo $cmd
        done
      done
    done
  done
}

function _gen_vary_size_jobs() {
  rep=10
  algs=(jps2 jps2-prune2)
  sizes=(256 512 1024 2048)
  datdir="./${dir}/vary-size"
  outdir="./synthetic-output/vary-size"
  for size in "${sizes[@]}"; do
    for alg in "${algs[@]}"; do
      for (( i=0; i<rep; i++)) {
        mfile="${datdir}/diag-${size}.map"
        mname="diag-${size}"
        sfile="${datdir}/diag-${size}.scen"
        exe="./build/fast/bin/warthog"
        outpath="${outdir}/${alg}/$i"
        mkdir -p "${outpath}"
        cmd="${exe} --scen ${sfile} --map ${mfile} --alg ${alg} > ${outpath}/${mname}.log"
        echo $cmd
      }
    done
  done
}

function vary_size_time() {
  _gen_vary_size_jobs | shuf > varysize.sh
  chmpd u+x varysize.sh
  bash -x varysize.sh
}

function vary_size_subcnt() {
  algs=(jps2 jps2-prune2)
  sizes=(256 512 1024 2048)
  datdir="./${dir}/vary-size"
  outdir="./synthetic-output/vary-size"
  for size in "${sizes[@]}"; do
    for alg in "${algs[@]}"; do
      mfile="${datdir}/diag-${size}.map"
      mname="diag-${size}"
      sfile="${datdir}/diag-${size}.scen"
      exe="./build/fast/bin/experiment"
      outpath="${outdir}/${alg}/$i"
      mkdir -p "${outpath}"
      cmd="${exe} ${mfile} ${sfile} subcnt > ${outpath}/${mname}.log"
      echo $cmd
      eval $cmd
    done
  done
}

function vary_len_time() {
  rep=10
  algs=(jps2 jps2-prune2)
  rs=(0.1 )
  drs=(0.75)
  lens=(256 512 1024 2048)
  sml_outdir="./synthetic-output/vary_len"
  _gen_jobs_time | shuf > varylen.sh
  chmod u+x varylen.sh
  bash -x varylen.sh
}

function vary_len_subcnt() {
  algs=(jps2 jps2-prune2)
  rs=(0.1)
  drs=(0.75)
  lens=(256 512 1024 2048)
  sml_outdir="./synthetic-output/vary_len"
  _gen_jbos_subcnt > varylen_subcnt.sh
  chmod u+x varylen_subcnt.sh
  bash -x varylen_subcnt.sh
}

function vary_r_time() {
  rep=10
  algs=(jps2 jps2-prune2)
  rs=(0 0.1 0.5 1 1.5 10 20)
  drs=(0.75)
  lens=(512)
  sml_outdir="./synthetic-output/vary_r"
  _gen_jobs_time | shuf > varyr.sh
  chmod u+x varyr.sh
  bash -x varyr.sh
}

function vary_r_subcnt() {
  algs=(jps2 jps2-prune2)
  rs=(0 0.1 0.5 1 1.5 10 20)
  drs=(0.75)
  lens=(512)
  sml_outdir="./synthetic-output/vary_r"
  _gen_jbos_subcnt > varyr_subcnt.sh
  chmod u+x varyr_subcnt.sh
  bash -x varyr_subcnt.sh
}

function vary_br_time() {
  rep=10
  algs=(jps2 jps2-prune2)
  rs=(0.1 )
  drs=(0 0.25 0.5 0.75 1)
  lens=(512)
  sml_outdir="./synthetic-output/vary_br"
  _gen_jobs_time | shuf > varybr.sh
  chmod u+x varybr.sh
  bash -x varybr.sh
}

function vary_br_subcnt() {
  algs=(jps2 jps2-prune2)
  rs=(0.1 )
  drs=(0 0.25 0.5 0.75 1)
  lens=(512)
  sml_outdir="./synthetic-output/vary_br"
  _gen_jbos_subcnt > varybr_subcnt.sh
  chmod u+x varybr_subcnt.sh
  bash -x varybr_subcnt.sh
}

function measure_time() {
  make clean && make fast -j
  vary_r_time
  vary_len_time
  vary_br_time
  vary_size_time
}

function measure_subopt() {
  # make clean && make fastcnt -j
  vary_r_subcnt
  vary_len_subcnt
  vary_br_subcnt
  vary_size_subcnt
}

case "$1" in
  gen) gen_maps ;;
  time) measure_time ;;
  sub) measure_subopt ;;
  *)
    echo $"Usage: $0 {gen|time|sub}"
    exit 1
esac
