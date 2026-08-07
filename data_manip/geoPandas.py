# !pip install geopandas
import geopandas as gpd

# example file names (edit to suit your needs)
path_shp = "GML/P21-12a_26.shp"
pt = "/point_n180.geojson"
hx = "/hex500_n60.geojson"
output = "/hogehoge/rslt.geojson"

# example reading in GeoJSON (joining pt and hx)
df_pt = gpd.read_file(pt)                                                  # Random points
df_hx = gpd.read_file(hx)                                                  # Hexagon polygons

# join the 2 layers above read from the geoJson files
spj = gpd.sjoin(df_pt, df_hx, op='within')
spj.head()

# write GeoJSON to a file specified by output
spj.to_file(output, driver="GeoJSON")

# example reading shp file
gdf = gpd.read_file(path_shp, encoding='shift-JIS')
gdf.head()

# !pip install geocoder
# !pip install leafmap
# !pip install PyCRS # for m_add_shp
import geocoder
import leafmap

# Specify the initial display range of the map
location = 'Kyoto Prefecture'
ret = geocoder.osm(location, timeout=5.0)
m = leafmap.Map(center=ret.latlng, zoom=9)

# Added the location of the Kyoto Prefecture waterworks utility.
m.add_shp(path_shp, layer_name="kyoto", encoding='shift-JIS')

# display
m