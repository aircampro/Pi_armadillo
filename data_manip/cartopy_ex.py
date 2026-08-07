import cartopy.crs as ccrs
import cartopy.feature as cfeature
import matplotlib.pyplot as plt
from ob_tran import GeneralObliqueProjection
from ob_tran_utils import *

if __name__ == "__main__":
    # Perform rotations around each axis to create the target rotation matrix
    r = Ry(-37) @ Rx(135) @ Ry(37) @ Rz(-137)

    # Derive parameters from the rotation matrix
    params = params_from_rotation(r)

    # Create the CRS by specifying the rotation parameters and the projection method (post-oblique transformation)
    # Mollweide projection with the Japanese archipelago inverted and positioned at mid-latitudes
    crs = GeneralObliqueProjection(
        ccrs.Mollweide(),
        central_longitude=params['lon_0'],
        pole_latitude=params['lat_p'], pole_longitude=params['lon_p'])

    # Construction (geometry)
    fig = plt.figure(figsize=(8, 4))
    ax = plt.axes(projection=crs)
    ax.spines['geo'].set(facecolor='white')
    ax.spines['geo'].set(zorder=0)

    lon_ticks = 15
    lat_ticks = 15
    ax.add_feature(
        cfeature.NaturalEarthFeature(
            category='cultural', scale='10m',
            name='admin_0_countries_lakes'),
        facecolor="#32CD32", edgecolor='black', linewidth=0.1)
    ax.gridlines(
        draw_labels=False, color='gray', linewidth=0.5, alpha=0.5, 
        xlocs=range(-180,180,lon_ticks),
        ylocs=range(-90+lat_ticks,90,lat_ticks))
    ax.gridlines(
        draw_labels=False, color='red', linewidth=0.5, alpha=0.5, 
        xlocs=[0, 180], ylocs=[0])

    fig.subplots_adjust(left=0.01, right=0.99, bottom=0.01, top=0.99)
    plt.savefig(f"./cartopy_example.png", dpi=300, transparent=True)
    plt.close()