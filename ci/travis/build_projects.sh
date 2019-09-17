#!/bin/bash

if [ "$TRAVIS_OS_NAME" == "linux" ] 
then 
	echo "Building ad9361 on Linux"
	make -C ./ad9361/sw -f Makefile.linux
fi

echo "Building generic ad9361"
make -C ./ad9361/sw -f Makefile.generic all
make -C ./ad9361/sw -f Makefile.generic clean

echo "Building no-OS drivers"
for file in `find ./drivers -type f -name "*.c"`; do
	OK=1
	while read line
	do
		[[ ${file} =~ ${line} ]] && OK=0	       
	done <  ./drivers/make/exclude
	[[ $OK != 0 ]]
	SOURCE_FILES="${SOURCE_FILES} ${file}"
       	
done

make -f ./drivers/make/Makefile VERBOSE=0 ARG1="${SOURCE_FILES}" all
make -f ./drivers/make/Makefile clean
