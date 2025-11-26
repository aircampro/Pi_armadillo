# kmeans clustering using scipy on precipitable_water read from a netCDF4 file
# 
import netCDF4
import numpy as np
from scipy.cluster.vq import *
from matplotlib import colors as c
import matplotlib.pyplot as plt

f = netCDF4.Dataset('tpw_v07r01_200910.nc4.nc', 'r')
lats = f.variables['latitude'][:]
lons = f.variables['longitude'][:]
pw = f.variables['precipitable_water'][0,:,:]

f.close()
# Flatten image to get line of values
flatraster = pw.flatten()
flatraster.mask = False
flatraster = flatraster.data

# Create figure to receive results
fig = plt.figure(figsize=[20,7])
fig.suptitle('K-Means Clustering')

# In first subplot add original image
ax = plt.subplot(241)
ax.axis('off')
ax.set_title('Original Image\nMonthly Average Precipitable Water\n over Ice-Free Oceans (kg m-2)')
original=ax.imshow(pw, cmap='rainbow', interpolation='nearest', aspect='auto', origin='lower')
plt.colorbar(original, cmap='rainbow', ax=ax, orientation='vertical')
# In remaining subplots add k-means clustered images
# Define colormap
list_colors=['blue','orange', 'green', 'magenta', 'cyan', 'gray', 'red', 'yellow']
for i in range(7):
    print("Calculate k-means with ", i+2, " clusters.")
    
    #This scipy code clusters k-mean, code has same length as flattened
    # raster and defines which cluster the value corresponds to
    centroids, variance = kmeans(flatraster.astype(float), i+2)
    code, distance = vq(flatraster, centroids)
    
    #Since code contains the clustered values, reshape into SAR dimensions
    codeim = code.reshape(pw.shape[0], pw.shape[1])
    
    #Plot the subplot with (i+2)th k-means
    ax = plt.subplot(2,4,i+2)
    ax.axis('off')
    xlabel = str(i+2) , ' clusters'
    ax.set_title(xlabel)
    bounds=range(0,i+2)
    cmap = c.ListedColormap(list_colors[0:i+2])
    kmp=ax.imshow(codeim, interpolation='nearest', aspect='auto', cmap=cmap,  origin='lower')
    plt.colorbar(kmp, cmap=cmap,  ticks=bounds, ax=ax, orientation='vertical')
plt.show()