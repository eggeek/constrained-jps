#!/bin/bash

filename=`date +"%s"`
filename="results_"$filename

./bin/roadhog --alg ch --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr.ch ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg ch-astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr.ch ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fch-bbaf --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr.ch ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc ./dimacs/USA-road-d.NY.gr.ch.fch-bbaf.arclabel ./dimacs/USA-road-d.NY.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch-bb --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr.ch ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc ./dimacs/USA-road-d.NY.gr.ch.fch-bb.arclabel ./dimacs/USA-road-d.NY.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch-af --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr.ch ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc ./dimacs/USA-road-d.NY.gr.ch.fch-af.arclabel ./dimacs/USA-road-d.NY.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr.ch ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr.ch ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg bi-astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg dijkstra --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg bi-dijkstra --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc | tee -a $filename


./bin/roadhog --noheader --alg ch --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr.ch ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg ch-astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr.ch ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fch-bbaf --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr.ch ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc ./dimacs/USA-road-d.BAY.gr.ch.fch-bbaf.arclabel ./dimacs/USA-road-d.BAY.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch-bb --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr.ch ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc ./dimacs/USA-road-d.BAY.gr.ch.fch-bb.arclabel ./dimacs/USA-road-d.BAY.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch-af --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr.ch ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc ./dimacs/USA-road-d.BAY.gr.ch.fch-af.arclabel ./dimacs/USA-road-d.BAY.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr.ch ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr.ch ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg bi-astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg dijkstra --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg bi-dijkstra --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc | tee -a $filename


./bin/roadhog --noheader --alg ch --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr.ch ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg ch-astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr.ch ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fch-bbaf --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr.ch ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc ./dimacs/USA-road-d.COL.gr.ch.fch-bbaf.arclabel ./dimacs/USA-road-d.COL.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch-bb --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr.ch ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc ./dimacs/USA-road-d.COL.gr.ch.fch-bb.arclabel ./dimacs/USA-road-d.COL.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch-af --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr.ch ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc ./dimacs/USA-road-d.COL.gr.ch.fch-af.arclabel ./dimacs/USA-road-d.COL.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr.ch ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr.ch ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg bi-astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg dijkstra --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg bi-dijkstra --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc | tee -a $filename


./bin/roadhog --noheader --alg ch --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr.ch ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg ch-astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr.ch ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fch-bbaf --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr.ch ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc ./dimacs/USA-road-d.FLA.gr.ch.fch-bbaf.arclabel ./dimacs/USA-road-d.FLA.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch-bb --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr.ch ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc ./dimacs/USA-road-d.FLA.gr.ch.fch-bb.arclabel ./dimacs/USA-road-d.FLA.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch-af --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr.ch ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc ./dimacs/USA-road-d.FLA.gr.ch.fch-af.arclabel ./dimacs/USA-road-d.FLA.gr.metis.part.128 | tee -a $filename

./bin/roadhog --noheader --alg fch --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr.ch ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr.ch ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg bi-astar --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg dijkstra --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg bi-dijkstra --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx no_paa --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr.ch ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx no_paa --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr.ch ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx no_paa --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr.ch ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx no_paa --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr.ch ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc | tee -a $filename 

./bin/roadhog --noheader --alg fchx no_pba --problem ./dimacs/inputs/USA-road-d/USA-road-d.NY.p2p --input ./dimacs/USA-road-d.NY.gr.ch ./dimacs/USA-road-d.NY.co ./dimacs/USA-road-d.NY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx no_pba --problem ./dimacs/inputs/USA-road-d/USA-road-d.BAY.p2p --input ./dimacs/USA-road-d.BAY.gr.ch ./dimacs/USA-road-d.BAY.co ./dimacs/USA-road-d.BAY.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx no_pba --problem ./dimacs/inputs/USA-road-d/USA-road-d.COL.p2p --input ./dimacs/USA-road-d.COL.gr.ch ./dimacs/USA-road-d.COL.co ./dimacs/USA-road-d.COL.gr.ooc | tee -a $filename

./bin/roadhog --noheader --alg fchx no_pba --problem ./dimacs/inputs/USA-road-d/USA-road-d.FLA.p2p --input ./dimacs/USA-road-d.FLA.gr.ch ./dimacs/USA-road-d.FLA.co ./dimacs/USA-road-d.FLA.gr.ooc | tee -a $filename 

