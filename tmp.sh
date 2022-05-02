#!/bin/bash

for input in $(ls | grep out)
do
	input_name=$(echo $input | cut -d '.' -f1)
	input_name="${input_name}.in"
	btb_size=$(head -n 1 $input_name | cut -d ' ' -f1)
	last_line=$(tail -n 1 $input)
	size_entry=$(echo $last_line | cut -d ' ' -f5)
	flush_num_entry=$(echo $last_line | cut -d ' ' -f1)
	flush_num=$(echo $last_line | cut -d ' ' -f2)
	br_num_entry=$(echo $last_line | cut -d ' ' -f3)
	br_num=$(echo $last_line | cut -d ' ' -f4)
	size=$(echo $last_line | cut -d ' ' -f6 | cut -d 'b' -f1)
	size=$(($size+$btb_size))
	size="${size}b"
	new_line="${flush_num_entry} ${flush_num} ${br_num_entry} ${br_num} ${size_entry} ${size}"
	echo $new_line
	sed -i '$ d' $input
	echo $new_line >> $input
done
