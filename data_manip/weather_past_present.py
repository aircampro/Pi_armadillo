#!/usr/bin/env python
# Use case: In the RCP5 scenario predicted by the MIROC5 model, to obtain a daily maximum temperature and DVI of 35°N and 135°E between 2020 and 2030. 
#
# Global Climate Model	MRI-CGCM3、MIROC5、CSIRO-Mk3-6-0*、GFDL-CM3*、HadGEM2-ES*
# Warming scenario	RCP 8.5, RCP 2.6
# Meteorological elements (symbols)	TMP_mea、TMP_max、TMP_min、APCP、GSR*、RH*、WIND* 
# https://github.com/ky0on/simriw/blob/master/AMD_Tools3.py
# https://github.com/ky0on/simriw/tree/master
# queries data from here https://amd.rd.naro.go.jp/opendap/AMD/
#
#
#
import AMD_Tools3 as AMD
import DayLength as DL                         
import datetime
import sys

# Para=[ Gv0, Th , A , Lc , B ,DVI*]
# Function name: ʼHorie et al. (1995)ʼ
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
# Function name: ʼNormalized Effective Degree Daysʼ
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
scenario_list = ['RCP 8.5', 'RCP 2.6']
# choose the scenrio e.g. RCP 8.5
scen_chose=0

model = model_list[model_chose]
scenario = scenario_list[scen_chose]
# choose date range for data
TODAY=datetime.datetime.today()
if sys.argc >= 4 :
    SDATE=TODAY - datetime.timedelta(days=int(sys.argv[3]))
    EDATE=TODAY + datetime.timedelta(days=int(sys.argv[4]))
else:
    SDATE=TODAY - datetime.timedelta(days=20)
    EDATE=TODAY + datetime.timedelta(days=15)
timedomain = [SDATE.strftime('%Y-%m-%d'), EDATE.strftime('%Y-%m-%d')]
# this is the latitude longditude domain for the data
if sys.argc >= 2 :
    lalodomain = [float(sys.argv[1]), float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[2])]
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

Tmea[Tmea.mask == True] = np.nan
Ld = DL.daylength(tim, lat, lon)                                          # Calculate the day length of the entire target space-time with the module daylengthd.
DATA_ON_USB = 0                                                           # data from usb-1 else server
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
                else: #出穂から成熟までは…
                    DVR = DVR12(Tmea[t-1,i,j])
                    DVI[t,i,j] = DVI[t-1,i,j] + DVR
                    if DVI[t-1,i,j] < 2.0 and DVI[t,i,j] >= 2.0: #収穫適日ならば…
                        DOR[i,j] = frst + np.timedelta64(t,'D') #日付を記録する．
DVI[np.ma.where(DVI > 2.0)] = 2.0 #全体を見直してみて，DVI が2より大きいメッシュがあれ

# write out an excel sheet with the tmax data appended to it
df = pd.read_excel("temperature.xlsx", index_col=0)
linking_df = pd.DataFrame({'Tmax':Tmax, 'time':tim0, 'lat':lat0, 'lon':lon0})
excel_df = pd.concat([df, linking_df], axis=1)
pd.io.formats.excel.ExcelFormatter.header_style = None
excel_df.to_excel('temperature.xlsx', sheet_name='test')

# write out an excel sheet with the DVI data appended to it
df = pd.read_excel("dvi.xlsx", index_col=0)
linking_df = pd.DataFrame({'DVI':DVI, 'time':tim, 'lat':lat0, 'lon':lon0})
excel_df = pd.concat([df, linking_df], axis=1)
pd.io.formats.excel.ExcelFormatter.header_style = None
name = str(datetime.datetime.now()).replace(' ','_').replace(":","_").replace(".","_").replace("-","_")
excel_df.to_excel(f'{name}(DVI).xlsx', sheet_name='dvi')