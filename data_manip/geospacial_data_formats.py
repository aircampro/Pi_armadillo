# geospacial data formats
# example netCDF4
#
from netCDF4 import Dataset
import sys
import numpy as np
import matplotlib.pyplot as plt

# read a file and show its contents
if len(sys.argv[0]) >= 1:
    nc_f=str(sys.argv[1])
else:
    nc_f = 'tpw_v07r01_200910.nc4.nc'  # Your filename
nc_fid = Dataset(nc_f, 'r')  # Dataset is the class behavior to open the file
                             # and create an instance of the ncCDF4 class
print(nc_fid)
nc_fid.close()

# write a file
f = netCDF4.Dataset('orography.nc', 'w')
f.createDimension('time', None)
f.createDimension('z', 3)
f.createDimension('y', 4)
f.createDimension('x', 5)
lats = f.createVariable('lat', float, ('y', ), zlib=True)
lons = f.createVariable('lon', float, ('x', ), zlib=True)
orography = f.createVariable('orog', float, ('y', 'x'), zlib=True, least_significant_digit=1, fill_value=0)
# create latitude and longitude 1D arrays
lat_out  = [60, 65, 70, 75]                                     # dim 4 y
lon_out  = [ 30,  60,  90, 120, 150]                            # dim 5 x
# Create field values for orography
data_out = np.arange(4*5)                                       # 1d array but with dimension x*y
data_out.shape = (4,5)                                          # reshape to 2d array
orography[:] = data_out
lats[:] = lat_out
lons[:] = lon_out
f.close()

# this is how you can read this back
f = netCDF4.Dataset('orography.nc', 'r')
lats = f.variables['lat']
lons = f.variables['lon']
orography = f.variables['orog']
print(lats[:])
print(lons[:])
print(orography[:][3,2])
f.close()

# geoTIFF formats on linux also check out gdalinfo command
from osgeo import gdal
datafile = gdal.Open('metos_python/data/Southern_Norway_and_Sweden.2017229.terra.1km.tif')
print( "Driver: ",datafile.GetDriver().ShortName, datafile.GetDriver().LongName)
print( "Size is ", datafile.RasterXSize, datafile.RasterYSize)
print( "Bands = ", datafile.RasterCount)
print( "Coordinate System is:", datafile.GetProjectionRef ())
print( "GetGeoTransform() = ", datafile.GetGeoTransform ())
print( "GetMetadata() = ", datafile.GetMetadata ())
# This GeoTIFF file contains 3 bands (Red, Green and Blue), so we can extract each band and visualize one band with imshow:
bnd1 = datafile.GetRasterBand(1).ReadAsArray()
bnd2 = datafile.GetRasterBand(2).ReadAsArray()
bnd3 = datafile.GetRasterBand(3).ReadAsArray()
plt.imshow(bnd1)
plt.show()
# now visualise them all by stacking the bands into one image
print(type(bnd1), bnd1.shape)
print(type(bnd2), bnd3.shape)
print(type(bnd3), bnd3.shape)
img = np.dstack((bnd1,bnd2,bnd3))
print(type(img), img.shape)
plt.imshow(img)
plt.show()
# Shapefile The most ubiquitous vector format is the ESRI shapefile. Geospatial Software company ESRI released the shapefile 
# format specification as an open format in 1998. It was initially developed for their ArcView software but it became quickly an unofficial GIS standard.
# Let’s take an example (downloaded from http://www.mapcruzin.com/free-norway-arcgis-maps-shapefiles.htm).
#
from osgeo import ogr
from mpl_toolkits.basemap import Basemap

fig = plt.figure(figsize=[12,15])  # a new figure window
ax = fig.add_subplot(1, 1, 1)  # specify (nrows, ncols, axnum)
ax.set_title('Cities in Norway', fontsize=14)

map = Basemap(llcrnrlon=-1.0,urcrnrlon=40.,llcrnrlat=55.,urcrnrlat=75., resolution='i', projection='lcc', lat_1=65., lon_0=5.)
map.drawmapboundary(fill_color='aqua')
map.fillcontinents(color='#ffe2ab',lake_color='aqua')
map.drawcoastlines()

shapedata = ogr.Open('Norway_places')
layer = shapedata.GetLayer()
for i in range(layer.GetFeatureCount()):
    feature = layer.GetFeature(i)
    name = feature.GetField("NAME")
    type = feature.GetField("TYPE")
    if type == 'city':
        geometry = feature.GetGeometryRef()
        lon = geometry.GetPoint()[0]
        lat = geometry.GetPoint()[1]
        x,y = map(lon,lat)
        map.plot(x, y, marker=marker, color='red', markersize=8, markeredgewidth=2)
        ax.annotate(name, (x, y), color='blue', fontsize=14)
plt.show()
# geoJSON  refer to RFC 7946. parsing
la = ogr.Open('la_city.geojson')
nblayer = la.GetLayerCount()
print("Number of layers: ", nblayer)
layer = la.GetLayer()
cities_us = []
for i in range(layer.GetFeatureCount()):
    feature = layer.GetFeature(i)
    name = feature.GetField("NAME")
    geometry = feature.GetGeometryRef()
    cities_us.append([i,name,geometry.GetGeometryName(), geometry.GetPoints()])

print(cities_us)
