#!/bin/bash
# map name
path=benchmark/32x32_obst204
files=$(ls $path)
base_program=build
declare -a algorithms
# algorithm file names
algorithms[0]=mapf_prioritized_sipp
algorithms[1]=cbs
algorithms[2]=ecbs
# MAPF
# create output directory
for filename in $files
do
	s1=${filename#*agents*}
	agvNum=${s1%_ex*}
	# directory name is AGV number
	if [ ! -d "output/MAPF/${agvNum}" ];then mkdir -p output/MAPF/${agvNum};fi
	# skip finished part
	if [ -f "output/MAPF/${agvNum}/${algorithms[0]}_${s1}" ];then
		echo 'result exists'
	else	
		${base_program}/${algorithms[0]} -i $path/$filename -o output/MAPF/${agvNum}/${algorithms[0]}_${s1} 	
	fi
done
echo '----------------MAPF finish----------------------'

# ECBS
for filename in $files
do
	s1=${filename#*agents*}
	agvNum=${s1%_ex*}
	if [ ! -d "output/ECBS/${agvNum}" ];then mkdir -p output/ECBS/${agvNum};fi
	if [ -f "output/ECBS/${agvNum}/${algorithms[2]}_${s1}" ];then
		echo 'result exists'
	else	
	 	timeout 30 ${base_program}/${algorithms[2]} -i $path/$filename -o output/ECBS/${agvNum}/${algorithms[2]}_${s1} -w 1.3
	fi
done
echo '-------------------ECBS finish-------------------------------'
for filename in $files
do
	s1=${filename#*agents*}
	agvNum=${s1%_ex*}
	if [ ! -d "output/CBS/${agvNum}" ];then mkdir -p output/CBS/${agvNum};fi
	if [ -f "output/CBS/${agvNum}/${algorithms[1]}_${s1}" ];then
		echo 'result exists'
	else	
	 	timeout 30 ${base_program}/${algorithms[1]} -i $path/$filename -o output/CBS/${agvNum}/${algorithms[1]}_${s1} 	
	fi
done
echo '---------------CBS finfish-------------------'

