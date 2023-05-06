#!/bin/bash
# 
if [ $# -lt 2 ]
then
    echo "required: [gr file] [range size]"
    exit 1;
fi

range_sz=$2
gr_nodes=`cat $1 | grep "p sp" | cut -d' '  -f3`;

i=0;
while [ $i -lt $gr_nodes ]
do
    if [ $i -gt $gr_nodes ]
    then
        $i=$gr_nodes;
    fi
    echo "$i $(($i + $range_sz))"
    i=$(($i+range_sz));
done
