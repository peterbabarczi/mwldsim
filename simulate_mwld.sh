#! /bin/csh 

for C in net/*.lgf
do

	for j in 1
	do
		for S in 0.0 
		do
			for typeA in 0.1 
			do
				for cnum in 0 1 2 3 4
				do
					for method in 12 
					do
						echo ./demo_restricted $C ${C%.lgf}_${S}.srg ${C%.lgf}_100_${j}.trf -cap_param 50000 -method $method -max_common_node_num $cnum -type_A_allowed_common_percentage ${typeA} -type_A_percentage ${typeA} -xml -comment _m${method}_ta${typeA}_cn${cnum}  
						./demo_restricted $C ${C%.lgf}_${S}.srg ${C%.lgf}_100_${j}.trf -cap_param 50000 -method $method -max_common_node_num $cnum -type_A_allowed_common_percentage ${typeA} -type_A_percentage  ${typeA} -xml -comment _m${method}_ta${typeA}_cn${cnum}
					done
				done
			done
		done
	done
	mv net/*.xml res/
done

