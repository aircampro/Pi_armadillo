#!/usr/bin/env python
# Use case: In the RCP5 scenario predicted by the MIROC5 model, to obtain a daily maximum temperature and DVI of 35°N and 135°E between 20 days before and 15 after this date
# or with the parameters specified as 4 command line arguments °N, °E, days_before, days_after
#
# Global Climate Model	MRI-CGCM3、MIROC5、CSIRO-Mk3-6-0*、GFDL-CM3*、HadGEM2-ES*
# Warming scenario	RCP 8.5, RCP 2.6
# Meteorological elements (symbols)	TMP_mea、TMP_max、TMP_min、APCP、GSR*、RH*、WIND* 
# https://github.com/ky0on/simriw/blob/master/AMD_Tools3.py
# https://github.com/ky0on/simriw/tree/master
# queries data from here https://amd.rd.naro.go.jp/opendap/AMD/
#
import AMD_Tools3 as AMD
import DayLength as DL                         
import datetime
import sys

import zipfile
from zipfile import ZipFile

# makes a zip file to upload to the ftp server
#
def make_zipfile(files_list, file_out='amd_data.zip'):
    with ZipFile(file_out, 'w') as zf:
        for f in files_list:
            zf.write(f)                                  

# uzip file which was zipped on the ftp server
#
def unload_zipfile(in_file='act_temp_data.zip'):
    with ZipFile(in_file,'r') as inputFile:
        inputFile.extractall()

# ftp client 
#
from ftplib import FTP
from datetime import datetime
import shutil

# connect to ftp server and perform the actions as spedified in the list
#
def ftp_client_actions(ftp_host = '10.0.1.5', ftp_user = 'root', ftp_pwd = 'passwdド', commands_list):

    with FTP(ftp_host, ftp_user, ftp_pwd) as ftp:
        for commnd in commands_list:
            if commnd == "change_udir":
	            ftp.cwd('/upload/')
            elif commnd == "change_ddir":
	            ftp.cwd('/download/')
            elif commnd == "get_files":
	            file_list = []
	            files_info = ftp.mlsd()
	            for filename, info in files_info:
                    if info['type'] == 'file':
                        modify_dt = datetime.strptime(info['modify'],"%Y%m%d%H%M%S")
                        file_list.append([filename,modify_dt])
	            latest_file = max(file_list, key = lambda x:x[1])
	            with open(latest_file[0], 'wb') as fp:
                    ftp.retrbinary(f'RETR {latest_file[0]}', fp.write)
            elif commnd == "put_files":
                f_upl_name = str(datetime.now()).replace(' ','_').replace('-','_').replace('.','_').replace(':','_') + ".zip"
                with open(f_upl_name, "wb") as f:
                    ftp.retrbinary("RETR /amd_data.zip", f.write)
                    
# function to extract geoJson from the Ministry of Land, Infrastructure, Transport and Tourism's administrative
#
# Data is pulled from the Ministry of Land, Infrastructure, Transport and Tourism's administrative area information
# Goal is to extract .geojson of the contents
import urllib
import urllib.request
import json

def get_mlit_info(url = 'https://nlftp.mlit.go.jp/ksj/gml/data/N03/N03-2019/N03-190101_47_GML.zip'):
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req) as res:
        data = res.read()
	
    # The data is in a zip file, so extract the contents and store them in a variable
    with zipfile.ZipFile(io.BytesIO(data), 'r') as zip_data:
        geojson_filepath = [name for name in zip_data.namelist() if '.geojson' in name][0]
        with zip_data.open(geojson_filepath, 'r') as gj:
            geojson_data = gj.read()
        geojson = json.loads(geojson_data)
        return geojson    

def create_json_zip(d = {'name': 'deep insider', 'addr': 'japan'}):
    content = json.dumps(d)
    print(content)  # {"name": "deep insider", "addr": "japan"}

    with ZipFile('mlit_data.zip', 'w') as zf:
        zf.writestr('data.bin', content) 
        
# Para=[ Gv0, Th , A , Lc , B ,DVI*]
# Function name: 'Horie et al. (1995)'
# Varid phase: emergence to heading
# Description:
# Gv0 [days]: the minimum number of days required for heading (GV)
# Th [C]: the temperature at which DVR is half the maximun rate at the optimum temperature (TH)
# A [1/C]: empilocal Parameter on air temperature (ALF)
# Lc [hour]: critical day length (LC)
# B [1/hour]: empilocal Parameter on day length (BDL)
# DVI* [ ]: DVI at which the crop becomes photosensitive (DVSAS).
# For KoshHikari;
# Gv0 , Th , A , Lc , B , DVI*
# 51.30, 17.80, 0.365, 16.00, 0.566, 0.230
#
# Gv0 A Th
def DVR01(DVI, Ta, Ld, Para=[51.3,17.8,0.365,16.0,0.566,0.23] ):
    FT = np.max( 1.0 / (Para[0] * (1.0 + np.exp(-Para[2] * (Ta - Para[1])))), 0.0)
    # B Lc
    FL = np.max((1.0 - np.exp(Para[4] * (Ld - Para[3]))), 0.0)
    DVR = FT
    # DVIs
    if DVI > Para[5] :
        DVR = FT * FL
    return DVR

def DVR12(Ta, Para=[0.0, 1000.0]):
# Para=[ T0, EDDd ]
# Function name: 'Normalized Effective Degree Days'
# Varid phase: not specified
# Description:
# T0 [degC]: threshold temperature of development
# EDDd [degC day]: desired effective degree days
# For the symple cumlative temperature of 1000[degC day];
# T0, EDDt
# 0.0, 1000.0
#
# T0 EDDd
    DVR = (Ta - Para[0]) / Para[1]
    return DVR

model_list = [MRI-CGCM3,、MIROC5,、CSIRO-Mk3-6-0,、GFDL-CM3,、HadGEM2-ES]
# choose the model e.g. MICROC5
model_chose = 1
scenario_list = ['RCP8.5', 'RCP2.6'] 
# choose the scenrio e.g. RCP 8.5
scen_chose=0

model = model_list[model_chose]
scenario = scenario_list[scen_chose]
# choose date range for data
TODAY=datetime.datetime.today()
if len(sys.argv) >= 4 :
    try:
        days_before = int(sys.argv[3])
        days_after = int(sys.argv[4])
    except ValueError:
        print("invalid parameters passed should be start_days end_days")
        sys.exit(-1)        
    SDATE=TODAY - datetime.timedelta(days=days_before)
    EDATE=TODAY + datetime.timedelta(days=days_after)
else:
    SDATE=TODAY - datetime.timedelta(days=20)
    EDATE=TODAY + datetime.timedelta(days=15)
timedomain = [SDATE.strftime('%Y-%m-%d'), EDATE.strftime('%Y-%m-%d')]
# this is the latitude longditude domain for the data
if len(sys.argv) >= 2 :
    try:
        lalodomain = [float(sys.argv[1]), float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[2])]
    except ValueError:
        print("invalid parameters passed should be °N °E")
        sys.exit(-1)
else:
    lalodomain = [35.0, 35.0, 135.0, 135.0]
# select the meterological element to get example temperature, humidity, wind speed etc
met_ele_list=[TMP_mea,、TMP_max,、TMP_min,、APCP,、GSR,、GLR, SSD, RH,、WIND]

area = 'Area3'
pref = 'pref_0800'                                                                  #茨城県

# call REST and get the data
# select measured temperature e.g. TMP_max
met_ele_sel = 1
Tmax, tim0, lat0, lon0 = AMD.GetData(met_ele_list[met_ele_sel], area, timedomain, lalodomain, model, scenario)

# select measured temperature e.g. TMP_mea to get DVI
met_ele_sel = 0
Tmea, tim, lat, lon = AMD.GetData(met_ele_list[met_ele_sel], area, timedomain, lalodomain, model, scenario)
Tacc = AMD.accumulation_of_effective_temperature( Ta, To=5.0 )
tacc_file = str(datetime.datetime.now()).split(' ')[0].replace('-','_')
AMD.PutNC_3D( Tacc, tim, lat, lon, description='Effective Degree Day Temperature', symbol='DDT', unit='degC day', filename=f'{tacc_file}_DDT.nc')

Tmea[Tmea.mask == True] = np.nan
Ld = DL.daylength(tim, lat, lon)                                          # Calculate the day length of the entire target space-time with the module daylengthd.
DATA_ON_USB = 0                                                           # data from usb=1 else server url='http://mesh.dc.affrc.go.jp/opendap to change set url variable
if DATA_ON_USB == 1:
    Pref, lat, lon = AMD.GetGeoData(pref, area, lalodomain, url='./AMD')  # When using prefecture data stored on a USB stick
else:．
    Pref, lat, lon = AMD.GetGeoData(pref, area, lalodomain)               # Use this to use prefecture data from a distribution server.

# Define an array to store the calculation results
frst = np.datetime64(timedomain[0])                                       # Numerical value of the first day of the period
last = np.datetime64(timedomain[1])                                       # Numerical value of the last day of the period
ntim = Tmea.shape[0]                                                      # number of days
nlat = Tmea.shape[1]                                                      # number of rows in the mesh
nlon = Tmea.shape[2]                                                      # number of columns in the mesh
DVI = np.ma.zeros((ntim, nlat, nlon))                                     # Array to record the spatio-temporal distribution of DVI (same spatio-temporal
#(same size as the weather data).
DVI[:,Pref==0.0] = np.ma.masked                                           # masked outside pefecture．
DVI[:,:,:] = DVI0                                                         # assign initial values to all regions.．
DOH = np.ma.zeros((nlat, nlon), dtype='datetime64[D]')                    # Array of days since the first day of the period.
DOH[Pref==0.0]                                                            # Array to store the date of ear emergence.
DOH[Pref==0.0] = np.ma.masked                                             # masked out of province
np.ma.harden_mask(DOH)                                                    # make the mask “hard” so that it does not change in subsequent calculations.．
DOH[:,:] = last                                                           # Put the last day of the period in all regions.
DOR = np.ma.zeros((nlat, nlon), dtype='datetime64[D]')                    # Array containing the best harvest date (number of days from the first day of the period).
# Array that will be used to determine the optimum harvest date (number of days from the first day of the period)
DOR[Pref==0.0] = np.ma.masked                                             # masked out of province.
np.ma.harden_mask(DOR)                                                    # make the mask “hard” so that it does not change in subsequent calculations.．
DOR[:,:] = last                                                           # Put the last day of the period in all regions.

# Calculate DVI for each mesh
for i in range(nlat):                                                     # Mesh rows (in latitude direction)
    for j in range(nlon):                                                 # Mesh rows (longitude direction)
        for t in range(1, ntim):                                          # number of days
            if DVI[t-1,i,j] < 1.0:                                        # from transplant to emergence...…
                DVR = DVR01(DVI[t-1,i,j], Tmea[t,i,j], Ld[t,i,j])
                DVI[t,i,j] = DVI[t-1,i,j] + DVR
                if DVI[t,i,j] >= 1.0:                                     # if outgoing date...
                    DOH[i,j] = frst + np.timedelta64(t,'D')               # Record the date.
                else:                                                     # From emergence to maturity...
                    DVR = DVR12(Tmea[t-1,i,j])
                    DVI[t,i,j] = DVI[t-1,i,j] + DVR
                    if DVI[t-1,i,j] < 2.0 and DVI[t,i,j] >= 2.0:          # If it is a good harvest day...…
                        DOR[i,j] = frst + np.timedelta64(t,'D')           # Record the date..
DVI[np.ma.where(DVI > 2.0)] = 2.0                                         # Review the entire area and if any mesh has DVI greater than 2

# write out an excel sheet with the tmax data appended to it
df = pd.read_excel("temperature.xlsx", index_col=0)                       # read previous
linking_df = pd.DataFrame({'Tmax':Tmax, 'time':tim0, 'lat':lat0, 'lon':lon0})
excel_df = pd.concat([df, linking_df], axis=1)
pd.io.formats.excel.ExcelFormatter.header_style = None
excel_df.to_excel('temperature.xlsx', sheet_name='temperture')            # append new data to previous file

# write out an excel sheet with the DVI data appended to it
df = pd.read_excel("dvi.xlsx", index_col=0)                               # read in the previous starting point
linking_df = pd.DataFrame({'DVI':DVI, 'time':tim, 'lat':lat, 'lon':lon})
excel_df = pd.concat([df, linking_df], axis=1)
pd.io.formats.excel.ExcelFormatter.header_style = None
name = str(datetime.datetime.now()).replace(' ','_').replace(":","_").replace(".","_").replace("-","_")
excel_df.to_excel(f'{name}_DVI.xlsx', sheet_name='dvi')                  # write out the new file data with appended data to a new file named after datetime
shutil.copyfile("dvi.xlsx", "dvi_backup.xlsx")
shutil.copyfile(f'{name}_DVI.xlsx', "dvi.xlsx")

# plot the data and save those plots as png files for upload to the ftp server 
import matplotlib.pyplot as plt
from matplotlib.dates import DateFormatter,DayLocator
import matplotlib.pylab as plt
import matplotlib.colors as clr
aspect = (lalodomain[3] - lalodomain[2]) / (lalodomain[1] - lalodomain[0]) + 0.5

fig = plt.figure(num=None, figsize=(5*aspect, 5))                          # Define figure object (container)
sclint = 3                                                                 # tick of color bar
sclmin = np.datetime64(SDATE)                                              # minimum value of color bar
sclmax = np.datetime64(EDATE)                                              # maximum value of color bar
levels = np.arange(sclmin, sclmax+np.timedelta64(sclint,'D'), sclint)
plt.axes(axisbg='0.8')                                                     # make background gray
cmap = plt.cm.Spectral                                                     # Colormap with nickname. Invert with “_r” at the end.
cmap.set_over('w', 1.0)                                                    # Color when the upper limit is exceeded
cmap.set_under('k', 1.0)                                                   # Color when the upper limit is exceeded
CF = plt.contourf(lon, lat, DOH, levels, cmap=cmap, extend='both')         # draw a distribution map
CB = plt.colorbar(CF, format=DateFormatter('%b %d'))                       # draw a color bar
plt.title('Date of Heading')                                               # title
plt.savefig('Date_of_Heading.png', dpi=600)                                # save the figure as a bitmap image
plt.show(fig)                                                              # display the figure
plt.clf()

fig = plt.figure(num=None, figsize=(5*aspect, 5))
sclint = 3
sclmin = np.datetime64(SDATE)
sclmax = np.datetime64(EDATE)
plt.axes(axisbg='0.8')
levels = np.arange(sclmin, sclmax+np.timedelta64(sclint,'D'), sclint)
cmap = plt.cm.Spectral
cmap.set_over('w', 1.0)
cmap.set_under('k', 1.0)
CF = plt.contourf(lon, lat, DOR, levels, cmap=cmap, extend='both')
CB = plt.colorbar(CF, format=DateFormatter('%b %d'))
plt.title('Date of Ripe')
plt.savefig('Date_of_Ripen.png', dpi=600)
plt.show(fig)
plt.clf()

# Save the calculation results to a file
AMD.PutNC_3D( DVI, tim, lat, lon, description='Rice Development Index', symbol='DVI', unit='-', filename='DVI.nc')

# get mlit info and make zip file from it to upload to the ftp server with the rest
gjd = get_mlit_info()
create_json_zip(gjd)

# zip this data and upload it, then download the actual data from the frp server and unzip it
zip_list = [ f'{name}_DVI.xlsx', 'temperature.xlsx', 'mlit_data.zip', 'Date_of_Heading.png', 'Date_of_Ripen.png', 'DVI.nc', f'{tacc_file}_DDT.nc' ]
make_zipfile( zip_list )
ftp_action_lst = [ "change_udir", "put_files", "change_ddir", "get_files" ]
ftp_client_actions( ftp_action_lst )

