# example of geojson and folium to make a map web page from the information given
# we will write out 3 maps in html with markers and boundaries as specified
# we shall then analyse a photo and put its location to the map
#
from geojson import Point, Feature, FeatureCollection, dump
# With folium, you can easily create a map from a Python program and view it on a web browser.
import folium
outfile = "out.geojson"

# for extracting co-ordinate info from the photograph
from PIL import Image
import PIL.ExifTags as ExifTags

def get_gps(fname):

    im = Image.open(fname)
    exif = {
        ExifTags.TAGS[k]: v
        for k, v in im._getexif().items()
        if k in ExifTags.TAGS
    }
    gps_tags = exif["GPSInfo"]
    gps = {
        ExifTags.GPSTAGS.get(t, t): gps_tags[t]
        for t in gps_tags
    }
    def conv_deg(v):
        d = float(v[0][0]) / float(v[0][1])
        m = float(v[1][0]) / float(v[1][1])
        s = float(v[2][0]) / float(v[2][1])
        return d + (m / 60.0) + (s / 3600.0)
    lat = conv_deg(gps["GPSLatitude"])
    lat_ref = gps["GPSLatitudeRef"]
    if lat_ref != "N": lat = 0 - lat
    lon = conv_deg(gps["GPSLongitude"])
    lon_ref = gps["GPSLongitudeRef"]
    if lon_ref != "E": lon = 0 - lon
    return lat, lon
    
# this is a example of markers to be used on the map
#
import pandas as pd
df = pd.DataFrame({
    'station': ['A', 'B', 'C', 'D', 'E'],
    'latitude': [10.777406835525145, 10.7574068357251454, 10.676406835725145, 10.777436835725145, 10.776406835725145],
    'longtude': [106.70299857740313, 106.70229858740313, 106.49999858740313, 106.68299858740313, 106.73299858740313],
})

# example of some map information to prepare as a geoJson 
#
PointsData = [{'name': 'Ho Chi Minh City Opera House',
               'latitude': 10.777406835725145,
               'longitude': 106.70299858740313,
               'color': '#f02600'},
              {'name': 'Independence Palace',
               'latitude': 10.778137451508647,
               'longitude': 106.69531332149265,
               'color': '#00ff96'}]

# Prepare Geojson FeatureCollection
ft_all = []
for i, p in enumerate(PointsData):
    lat, lon = p['latitude'], p['longitude']
    ft = Feature(geometry = Point((lon, lat,)), properties = {'name': p['name'], 'marker-color': p['color'], 'marker-size': 'medium','marker-symbol': ''})
    ft_all.append(ft)
ft_colct = FeatureCollection(ft_all)

# write out the geojson file
with open(outfile, 'w') as f:
    dump(ft_colct, f, indent=2)

# get the street map for the area being represented
coords = [ lat, lon ]                                                    # take last location we read for the map center
m = folium.Map(location=coords, zoom_start=15, width=800, height=600, tiles='openstreetmap') 
folium.GeoJson(outfile, name='region_name',
              style_function = lambda x: {
                                        'fillOpacity': 0.5,
                                        'fillColor': 'Orange',
                                        'color': 'Red'
              }).add_to(m)

# add markers from the point data
tooltip = "Click me!"

for p in PointsData:
    lat, lon = p['latitude'], p['longitude']
    folium.Marker(
        location=[lat, lon],
        popup="<i>info</i>",  
        tooltip=tooltip,  
        icon=folium.Icon(color="red", icon="tower")  
    ).add_to(m)

folium.LayerControl().add_to(m) 
m.save('street_basic.html') 

# add the points from the data frame to this map e.g. station = A,B etc..
#
for i, row in df.iterrows():
    folium.Marker(
        location=[row['latitude'], row['longtude']],
        popup=row['station'],
        icon=folium.Icon(color='blue', icon='home')
    ).add_to(m)
m.save('street_hotspts.html') 

# now use another terrain for the map and show the lat,lon co-ordinates
#
folium_map = folium.Map(location=coords, zoom_start=12, width=800, height=600, tiles="Stamen Terrain")
folium.GeoJson(outfile, name='region_name',
              style_function = lambda x: {
                                        'fillOpacity': 0.5,
                                        'fillColor': 'Orange',
                                        'color': 'Red'
              }).add_to(folium_map)
folium_map.add_child(folium.LatLngPopup())

# now add two different markers
folium.Marker(
    location=[10.775137451508647, 106.67631332149265],
    popup="<i>tower</i>",  
    tooltip=tooltip,  
    icon=folium.Icon(color="green", icon="tower")  
).add_to(folium_map)

folium.Marker(
    location=[10.675137451508647, 106.717631332149265],
    popup="<i>cloud</i>",  
    tooltip=tooltip,  
    icon=folium.Icon(color="green", icon="cloud")  
).add_to(folium_map)

# now add some boundaries using a geoJson 
geoJ = {
  "type": "Feature",
  "geometry": {
    "type": "MultiPolygon",
    "coordinates": [
      [
        [
          [10.797406835725145,106.73299858740313],
          [10.757406835725145,106.74599858740313],
          [10.767406835725145,106.73999858740313],
          [10.797406835725145,106.73299858740313]
        ]
      ],
      [
        [
          [10.797406835725145,106.73299858740313],
          [10.757406835725145,106.74599858740313],
          [10.767406835725145,106.73999858740313],
          [10.797406835725145,106.73299858740313]
        ]
      ]
    ]
  },
  "properties": {
    "name": "Region of Interest"
  }
}
folium.GeoJson(geoJ).add_to(folium_map)

# now put on the map the photo location co-ordinates
PHOTO_N="test.jpg"
plat, plon = get_gps(PHOTO_N)
folium.Marker(
    location=[plat, plon],
    popup="<i>photo location</i>",  
    tooltip=tooltip,  
    icon=folium.Icon(color="green", icon="home")  
).add_to(folium_map)

#save this version
folium_map.save('stamen_boundaries.html')