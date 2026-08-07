#
# example using gdal library on tiff file
#
from osgeo import gdal, gdalconst

if __name__ == "__main__":
    print("gdal version", gdal.VersionInfo())
    file_name = '/app/image/1_index_ndvi.tif'
    src = gdal.Open(file_name,
                    gdalconst.GA_ReadOnly)  # TIFF loading (read-only)
    print(type(src))                        # "osgeo.gdal.Dataset"