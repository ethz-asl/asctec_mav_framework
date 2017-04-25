#!/bin/bash -e

rm -rf /tmp/arm7-elf-gcc
mkdir /tmp/arm7-elf-gcc
cd /tmp/arm7-elf-gcc
curl -S http://www.asctec.de/downloads/software/sdk/arm7-gcc-2012-03-28-bin.tar.bz2 > arm7-gcc-2012-03-28-bin.tar.bz2
tar -xvf arm7-gcc-2012-03-28-bin.tar.bz2

sudo apt-get install -y libx11-6 libx11-dev libncurses5-dev libncursesw5-dbg libncursesw5-dev libncurses5 libncurses5-dbg libncursesw5  libnewlib-dev libnewlib-dev newlib-source  zlibc texinfo flex bison gdb

sudo apt-get install -y lib32z1 lib32bz2-1.0

tar -xvf arm7-gcc-2012-03-28.tar.bz2

cd usr/local/arm7/bin

for i in * 
do
   chmod a+x $i
done

#export PATH=$PATH:/tmp/arm7-elf-gcc/usr/local/arm7/bin
