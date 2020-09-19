#!/bin/sh

name=random-32-32-20
map=../MAPF-instances/mapf-map/$name.map
scen=../MAPF-instances/mapf-scen-random/scen-random/$name-random
output=../ees/exp/$name-random
for k in $(seq 15 15 150)
do
	for i in $(seq 1 1 25)
	do
		echo $k agents on instance $name-random-$i with PBS
		./pbs -m $map -a $scen-$i.scen -k $k -o $output-$k-PBS.csv -f 0
	done
done

