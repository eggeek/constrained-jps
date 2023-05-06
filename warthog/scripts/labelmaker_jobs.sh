#!/bin/bash

EINVAL=22;
if [ $# -lt 3 ]
then
    echo "Usage: [labelling type] [gr file] [co file] [job size (#nodes)]"
    exit $EINVAL;
fi;

LAB_TYPE=$1
GR_FILE=$2
CO_FILE=$3
JOB_SIZE=$4

exec ./jobs.sh $GR_FILE $JOB_SIZE | tr '\n' '\0' | xargs -0 -n 1 -I foo echo "./bin/labelmaker --type $LAB_TYPE foo $GR_FILE $CO_FILE"
