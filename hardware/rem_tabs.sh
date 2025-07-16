# simple script to sunstitute the leading tabs for 4 spaces in python files
#
for input in `ls *`
do
 expand -i -t 4 $input > ../untabbed/$input
done