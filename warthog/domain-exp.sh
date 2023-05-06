#!/bin/bash
domains=(
  ./maps/bgmaps
  # ./maps/iron
  # ./maps/random10
  # ./maps/starcraft
  # ./maps/street
  # ./maps/dao
  # ./maps/maze512
  # ./maps/rooms
)
rs=(
  0.1 
  0.5 
  # 1
  # 1.5
)


algs=(
  jps2
  jps2-prune2
)
out_dir="./domain-output"

function run_domain() {
  domain=$1
  dname=$(basename -- $domain)
  for mpath in `ls ${domain}/*.map`; do
    mapname=$(basename -- $mpath)
    spath="./scenarios/movingai/${dname}/${mapname}.scen"

    for alg in "${algs[@]}"; do
      outpath="${out_dir}/$alg"
      mkdir -p ${outpath}
      cmd="./bin/warthog --scen ${spath} --map ${mpath} --alg $alg > $outpath/$mapname.log"
      echo $cmd
      eval "$cmd"
    done
  done
}

function suboptcnt_per_query_gen() {
  datasets=(
    "./maps/"
    "./domain-data/maps-randomlized-0.1p"
    "./domain-data/maps-randomlized-0.5p"
    # "./domain-data/maps-randomlized-1p"
    # "./domain-data/maps-randomlized-1.5p"
  )

  outdirs=(
    "./domain-output/subcnt-0p"
    "./domain-output/subcnt-0.1p"
    "./domain-output/subcnt-0.5p"
    # "./domain-output/subcnt-1p"
    # "./domain-output/subcnt-1.5p"
  )

  for (( i=0; i<${#datasets[@]}; i++ )); do

    for domain in "${domains[@]}"; do
      dname=$(basename -- $domain)
      out_dir="${outdirs[$i]}"
      for mpath in "${datasets[$i]}/${dname}"/*.map; do
        mapname=$(basename "$mpath" .map)
        spath="./scenarios/movingai/${dname}/${mapname}.map.scen"

        for alg in "${algs[@]}"; do
          outpath="${out_dir}/${dname}"
          mkdir -p "${outpath}"
          cmd="./build/fast/bin/experiment ${mpath} ${spath} subcnt > ${outpath}/$mapname.log"
          echo $cmd
        done
      done
    done

  done

}

function suboptcnt_per_query() {
  suboptcnt_per_query_gen > subcnt_per_query.sh
  chmod u+x subcnt_per_query.sh
}

function _genjobs() {
  rep=10
  for (( i=0; i<rep; i++ )) {
    for dm in "${domains[@]}"; do
      for r in "${rs[@]}"; do
        domain=$(basename ${dm})
        rand_domain="./domain-data/maps-randomlized-${r}p/${domain}"
        for map in `ls ${rand_domain}/*.map`; do
          for alg in "${algs[@]}"; do
            mapname=$(basename ${map} .map)
            scenfile="./scenarios/movingai/${domain}/${mapname}.map.scen"
            if [[ ! -e ${scenfile} ]]; then
              echo "scenfile missing, map: ${map}"
              continue
            fi
            outpath="${out_dir}/maps-randomlized-${r}p/${domain}/${alg}/$i"
            mkdir -p ${outpath}
            cmd="./build/fast/bin/warthog --scen ${scenfile} --map ${map} --alg $alg > ${outpath}/${mapname}.log"
            echo $cmd
          done
        done
      done
    done
  }
}

function genjobs() {
  _genjobs | shuf > domain-jobs.sh
  chmod u+x domain-jobs.sh
}

function gen_rand() {
  for dm in "${domains[@]}"; do
    for map in `ls ${dm}/*.map`; do
      for r in "${rs[@]}"; do
        domain=$(basename ${dm})
        outpath="./domain-data/maps-randomlized-${r}p/${domain}"
        mkdir -p ${outpath}
        mapname=$(basename ${map} .map)
        scenfile="./scenarios/movingai/${domain}/${mapname}.map.scen"
        if [[ ! -e ${scenfile} ]]; then
          echo "scenfile missing, map: ${map}"
          continue
        fi
        mkdir -p $outpath
        rval=$(python -c "print(${r} / 100.0)")
        cmd="./gen.py rand_scen ${map} ${scenfile} ${rval} > ${outpath}/${mapname}.map"
        echo $cmd
        eval $cmd
      done
    done
  done
}

case "$1" in
  genjobs) genjobs;;
  gen_rand) gen_rand ;;
  gen_subq) suboptcnt_per_query;;
  # gen_subq) suboptcnt_per_query_gen;;
  *)
    echo $"Usage: $0 {exp|gen}"
    exit 1
esac
