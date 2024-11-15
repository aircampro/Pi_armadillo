# $pip install pulp ortoolpy 
# two dimensional packing solver
# PackingClass( width, height, items
#
from ortoolpy import TwoDimPackingClass
TwoDimPackingClass(500, 300, [(240, 150), (260, 100), \
    (100, 200), (240, 150), (160, 200)]).solve()
	
# makes pandas.DataFrame
from ortoolpy.optimization import TwoDimPacking
TwoDimPacking('data/packing1.csv', 500, 300)[1]

TwoDimPackingClass(500, 300, [(240, 100), (260, 110), \
    (150, 200), (240, 150), (160, 200), (200, 134), (110,200)]).solve()
TwoDimPacking('data/packing2.csv', 500, 300)[1]