#!/bin/bash -f

# Loop script for running ICP on large datasets. It divides the area of interest into 1 x 1 km square panels and runs ICP separately on each one.


#    Copyright (C) 2012  ASTRIL, ASU (http://robotics.asu.edu)
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Affero General Public License as
#    published by the Free Software Foundation, either version 3 of the
#    License, or (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Affero General Public License for more details.
#
#    You should have received a copy of the GNU Affero General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>
#
#    Author - Edwin Nissen (edwin.nissen@asu.edu)


# Enter the name of the full -pre and -post dataset .xyz files here. e.g. for mexico-pre.xyz and mexico-post.xyz, type mexico.
name=mexico
echo panel name is $name

# Enter corner UTM coordinates of the area of interest here (y first, then x). The middle 1000 refers to the panel dimensions, also in meters, and can be changed.
for blcy in $(seq 3574000 1000 3581000)
do
for blcx in $(seq 648000 1000 651000)
do

echo the left hand edge is $blcx
trcx=$(($blcx + 1000)) 
echo the right hand edge is $trcx
echo the bottom edge is $blcy
trcy=$(($blcy + 1000)) 
echo the top edge is $trcy

# Strip out points within the 1 x 1 km panel from the full pre- and post- xyz files 
echo "awk -v startx="$blcx" -v starty="$blcy" -v endx="$trcx" -v endy="$trcy" '$1>=startx && $1<=endx && $2>=starty && $2<=endy {print $0}' $name-pre.xyz > xyzfiles/$name-pre-$blcx-$blcy.xyz "
awk -v startx="$blcx" -v starty="$blcy" -v endx="$trcx" -v endy="$trcy" '$1>=startx && $1<=endx && $2>=starty && $2<=endy {print $0}' $name-pre.xyz > xyzfiles/$name-pre-$blcx-$blcy.xyz 

echo "awk -v startx="$blcx" -v starty="$blcy" -v endx="$trcx" -v endy="$trcy" '$1>=startx && $1<=endx && $2>=starty && $2<=endy {print $0}' $name-post.xyz > xyzfiles/$name-post-$blcx-$blcy.xyz "
awk -v startx="$blcx" -v starty="$blcy" -v endx="$trcx" -v endy="$trcy" '$1>=startx && $1<=endx && $2>=starty && $2<=endy {print $0}' $name-post.xyz > xyzfiles/$name-post-$blcx-$blcy.xyz 

# Add the PCL header to the .xyz panel file, skipping to the next panel for those which contain no (post-) data
numlines=$(wc -l < xyzfiles/$name-post-$blcx-$blcy.xyz)
echo the number of lines in the post .xyz file is $numlines
if [ $numlines -eq 0 ]
then continue
fi

echo "../texttopcd -i xyzfiles/$name-pre-$blcx-$blcy.xyz -s " " -o pcdfiles/$name-pre-$blcx-$blcy.pcd -x $blcx -y $blcy "
../texttopcd -i xyzfiles/$name-pre-$blcx-$blcy.xyz -s " " -o pcdfiles/$name-pre-$blcx-$blcy.pcd -x $blcx -y $blcy

echo "../texttopcd -i xyzfiles/$name-post-$blcx-$blcy.xyz -s " " -o pcdfiles/$name-post-$blcx-$blcy.pcd -x $blcx -y $blcy "
../texttopcd -i xyzfiles/$name-post-$blcx-$blcy.xyz -s " " -o pcdfiles/$name-post-$blcx-$blcy.pcd -x $blcx -y $blcy

# Run ICP (choosing which parameters to use) and get the output displacements

mkdir x

../registration -i pcdfiles/$name-pre-$blcx-$blcy.pcd -i pcdfiles/$name-post-$blcx-$blcy.pcd -t 5 -e 0.0001 --wx 200 --wy 200 --dx 10 --dy 10 | tee logfiles/log-$name-$blcx-$blcy.txt

../get_displacements -i logfiles/log-$name-$blcx-$blcy.txt -o disfiles/dis-$name-$blcx-$blcy.txt  -b x/window- -x 0 -y 0

mv x x-$name-$blcx-$blcy

echo "awk -v startx="$blcx" -v starty="$blcy" '{printf "%6.2f %7.2f %1.5f %1.5f %1.5f %1.5f %1.5f %1.5f\n", ($1+startx), ($2+starty), $3, $4, $5, $6, $7, $8}' disfiles/dis-$name-$blcx-$blcy.txt > geofiles/geo-$name-$blcx-$blcy.xyabcenz "
awk -v startx="$blcx" -v starty="$blcy" '{printf "%6.2f %7.2f %1.5f %1.5f %1.5f %1.5f %1.5f %1.5f\n", ($1+startx), ($2+starty), $3, $4, $5, $6, $7, $8}' disfiles/dis-$name-$blcx-$blcy.txt > geofiles/geo-$name-$blcx-$blcy.xyabcenz

# Add the displacements from the current panel into a single file containing all the displacements

echo "cat geofiles/all-$name.xyabcenz geofiles/geo-$name-$blcx-$blcy.xyabcenz > geofiles/temp.xyabcenz "
cat geofiles/all-$name.xyabcenz geofiles/geo-$name-$blcx-$blcy.xyabcenz > geofiles/temp.xyabcenz

echo "mv geofiles/temp.xyabcenz geofiles/all-$name.xyabcenz "
mv geofiles/temp.xyabcenz geofiles/all-$name.xyabcenz

# Move onto the next panel: either over (E) to the next column or up (N) to the westernmost panel on the next row. 

echo moving over to next column

done 

echo moving up to next row

done
#EOF
