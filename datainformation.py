import requests
import json
import arrow

# we can also define structures using ctypes rather than store our outputs in tuples
import ctypes
class SEA_LVL_JSON_T(Structure):
    __fields__ = [('sg', c_float), ('time', c_size_t)]
class WAVE_INF_JSON_T(Structure):
    __fields__ = [('smhi', c_float), ('time', c_size_t), ('noaa', c_float), ('meteo', c_float)]
class WIND_SPD_JSON_T(Structure):
    __fields__ = [('ws', c_float), ('time', c_size_t)]
class ELEVATION_JSON_T(Structure):
    __fields__ = [('elev', c_float), ('unit', c_wchar_p), ('time', c_size_t)]
class COMPOSITION_JSON_T():    
    iron = (0,0)                                # sg,mercator in tuple
    chlorophyll = (0,0)
    soilTemperature100cm = (0,0)
    soilTemperature40cm = (0,0)
    soilTemperature10cm = (0,0)
    soilTemperature = (0,0)
    soilMoisture100cm = (0,0)
    soilMoisture40cm = (0,0)
    soilMoisture10cm = (0,0)
    soilMoisture = (0,0) 
    nitrate = (0,0)
    phyto = (0,0)
    oxygen = (0,0)
    ph = (0,0)
    phytoplankton = (0,0)
    phosphate = (0,0)
    silicate = (0,0)
    salinity = (0,0)
COMP_SG=0
COMP_MERCATOR=1
   
class getExternalDataFromREST():

    # uk government flood monitoring stations/
    #
    def getDataFloodMonitor(stat="E72639", data_good_mins=15):
        api = "http://environment.data.gov.uk/flood-monitoring/id/stations/{station_id}"
        flood_url = api.format(station_id = city_name)
	    current_flood_value = -1
        headers = {'content-type': 'application/json'}
        try:
            response = requests.get(flood_url, headers=headers)
            response.raise_for_status()
        except requests.exceptions.HTTPError as errh:
            print ("Http Error:",errh)
        except requests.exceptions.ConnectionError as errc:
            print ("Error Connecting:",errc)
        except requests.exceptions.Timeout as errt:
            print ("Timeout Error:",errt)
        except requests.exceptions.RequestException as err:
            print ("OOps: Something Else",err)
        json_returned = json.dumps(response.json(), indent=4)
        flood_value = json_returned['items']['measures']['latestReading']['value']
		# format specifiers for the date time strings returned 
        DATE_PART=0
        TIME_PART=1
	HOUR_PART=0
	MIN_PART=1
	SEC_PART=2
	YR_PART=0
	MON_PART=1
	DAY_PART=2
        date_time_string = json_returned['items']['measures']['latestReading']['dateTime'].split("T")
        date_string = date_time_string[DATE_PART].split("-")
        time_string = date_time_string[TIME_PART].split(":")
	current_ts = str(datetime.datetime.now(pytz.timezone('Europe/Moscow'))) 
	current_yr = int(current_ts[YR_PART])
	current_mo = int(current_ts[MON_PART])		
	current_tmp = current_ts[DAY_PART].split(" ")
	current_day = int(current_tmp[0])
	current_time = current_tmp[1].split("+")[0].split(":")
	current_hr = int(current_time[HOUR_PART])
	current_min = int(current_time[MIN_PART])		
	current_sec = int(current_time[SEC_PART])
	if (((current_yr == int(date_string[YR_PART])) && (current_mo == int(date_string[MON_PART]))) && (current_day == int(date_string[DAY_PART]))):
	    if ((current_hr == int(time_string[HOUR_PART])) && (abs(current_min - int(time_string[MIN_PART])) < data_good_mins)):                   # data is current within (data_good) mins
	        current_flood_value = flood_value
	return current_flood_value

    def getOpenWeather(city_name="Larne",api_key="your key"):				
        api = "http://api.openweathermap.org/data/2.5/weather?units=metric&q={city}&APPID={key}"
        url = api.format(city = city_name, key = api_key)
        print(url)
        response = requests.get(url)
        jsonText = json.dumps(response.json(), indent=4)
        if city_name.find(str(jsonText['name'])) == -1:
            jsonText = { "incorrect_name" : str(jsonText['name']) }
        print(jsonText)
        return jsonText

    def parseOpenWeather(jsonText):
        lat=float(jsonText['coord']['lon'])
        lon=float(jsonText['coord']['lat'])        
        desc=str(jsonText['weather']['description'])       
        temp=float(jsonText['main']['temp'])
        press=float(jsonText['main']['pressure'])
        hum=float(jsonText['main']['humidity'])
        min_t=float(jsonText['main']['temp_min'])
        max_t=float(jsonText['main']['temp_max'])      
        vis=float(jsonText['visibility'])  
        wind_spd=float(jsonText['wind']['speed'])
        wind_deg=float(jsonText['wind']['deg'])    
        cloud=int(jsonText['clouds']['all'])   
        dt=int(jsonText['dt'])   
        rise=int(jsonText['sys']['sunrise'])  
        set=int(jsonText['sys']['sunset'])    
        return lat,lon,desc,temp,press,hum,min_t,max_t,vis,wind_spd,wind_deg,cloud,dt,rise,set

    # defines the tuple storage order for tidal info - in this example i'm storing the output data in a tuple array defined as below
    ht_idx=0
    time_idx=1
    type_idx=2  
    
    def getTideStormglass(lat_v=60.936, lon_v=5.114, api_v='example-api-key'):
        start = arrow.now().floor('day')                                                      # 0:00
        end = arrow.now().shift(days=1).floor('day')                                          # 23:59:59.999
        u='https://api.stormglass.io/v2/tide/extremes/point'
        p={'lat': lat_v,'lng': lon_v, 'start': start.to('UTC').timestamp(),  'end': end.to('UTC').timestamp(), }
        h={ 'Authorization': api_v }
        response = requests.get(u, params=p, headers=h)
        json_data = json.dumps(response.json())
        tidal_info=[]                                # this is an array of tuples in the format (height,time,type) as shown by the above idx names
        for ele in range(len(json_data['data'])):
            ht_datapoint = json_data['data'][ele]['height']
            time_datapoint = json_data['data'][ele]['time']
            type_datapoint = json_data['data'][ele]['type']
            tidal_info.append((ht_datapoint,time_datapoint,type_datapoint))
        lat = json_data['meta']['lat']
        lon = json_data['meta']['lng']
        return tidal_info,lat,lon
        
    def getTideValue(tidal_info_array,idx=time_idx,rec_num=0):
        if ((idx >= 3) || (rec_num>=len(tidal_info_array))) :
            return -1
        else:            
            return tidal_info_array[rec_num][idx]
 
    # in this example we are storing the output into a c like structure class SEA_LVL_JSON_T
    def getSeaLevelStormglass(lat_v=43.38, lon_v=-3.01, api_v='example-api-key'):
        start = arrow.now().floor('day')                                                      # 0:00
        end = arrow.now().shift(days=1).floor('day')                                          # 23:59:59.999
        u='https://api.stormglass.io/v2/tide/sea-level/point'
        p={'lat': lat_v,'lng': lon_v, 'start': start.to('UTC').timestamp(),  'end': end.to('UTC').timestamp(), }
        h={ 'Authorization': api_v }
        response = requests.get(u, params=p, headers=h)
        json_data = json.dumps(response.json())
        sea_lvl_info=[]                                # this is an array of tuples in the format (height,time,type) as shown by the above idx names
        data_s=SEA_LVL_JSON_T()
        for ele in range(len(json_data['data'])):
            data_s.sg = float(json_data['data'][ele]['sg'])
            data_s.time = json_data['data'][ele]['time']
            sea_lvl_info.append(data_s)
        lat = json_data['meta']['lat']
        lon = json_data['meta']['lng']
        return sea_lvl_info,lat,lon

    def getWindSpeedStormglass(lat_v=43.38, lon_v=-3.01, api_v='example-api-key'):
        start = arrow.now().floor('day')                                                      # 0:00
        end = arrow.now().shift(days=1).floor('day')                                          # 23:59:59.999
        u='https://api.stormglass.io/v2/weather/point'
        p={'lat': lat_v,'lng': lon_v, 'params': 'windSpeed', 'start': start.to('UTC').timestamp(),  'end': end.to('UTC').timestamp(), }
        h={ 'Authorization': api_v }
        response = requests.get(u, params=p, headers=h)
        json_data = json.dumps(response.json())
        wind_speed_info=[]                                # this is an array of tuples in the format (height,time,type) as shown by the above idx names
        data_s=WIND_SPD_JSON_T()
        for ele in range(len(json_data['data'])):
            data_s.ws = float(json_data['data'][ele]['windSpeed'])
            data_s.time = json_data['data'][ele]['time']
            wind_speed_info.append(data_s)
        lat = json_data['meta']['lat']
        lon = json_data['meta']['lng']
        return wind_speed_info,lat,lon

    def getElevastionStormglass(lat_v=43.38, lon_v=-3.01, api_v='example-api-key'):
        start = arrow.now().floor('day')                                                      # 0:00
        end = arrow.now().shift(days=1).floor('day')                                          # 23:59:59.999
        u='https://api.stormglass.io/v2/elevation/point'
        p={'lat': lat_v,'lng': lon_v, 'start': start.to('UTC').timestamp(),  'end': end.to('UTC').timestamp(), }
        h={ 'Authorization': api_v }
        response = requests.get(u, params=p, headers=h)
        json_data = json.dumps(response.json())
        elev_info=[]                                # this is an array of tuples in the format (height,time,type) as shown by the above idx names
        data_s=ELEVATION_JSON_T()
        for ele in range(len(json_data['data'])):
            data_s.elev = float(json_data['data'][ele]['elevation'])
            data_s.time = json_data['data'][ele]['time']
            data_s.unit = float(json_data['meta']['unit'])
            elev_info.append(data_s)
        lat = json_data['meta']['lat']
        lon = json_data['meta']['lng']
        return elev_info,lat,lon
        
    def getWaveInfoStormglass(lat_v=43.38, lon_v=-3.01, api_v='example-api-key'):
        start = arrow.now().floor('day')                                                      # 0:00
        end = arrow.now().shift(days=1).floor('day')                                          # 23:59:59.999
        u='https://api.stormglass.io/v2/weather/point'
        p={'lat': lat_v,'lng': lon_v, 'params': ','.join(['waveHeight', 'airTemperature']), 'start': start.to('UTC').timestamp(),  'end': end.to('UTC').timestamp(), }
        h={ 'Authorization': api_v }
        response = requests.get(u, params=p, headers=h)
        json_data = json.dumps(response.json())
        wave_info=[]                                # this is an array of tuples in the format (height,time,type) as shown by the above idx names
        data_s=WAVE_INF_JSON_T()
        for ele in range(len(json_data['hours'])):
            data_s.time = json_data['hours'][ele]['time']
            data_s.smhi = float(json_data['hours'][ele]['airTemperature']['smhi'])
            data_s.noaa = float(json_data['hours'][0]['waveHeight']['noaa'])
            data_s.meteo = float(json_data['hours'][0]['waveHeight']['meteo'])           
            wave_info.append(data_s)
        lat = json_data['meta']['lat']
        lon = json_data['meta']['lng']
        return wave_info,lat,lon

    def getUvIndexStormglass(lat_v=43.38, lon_v=-3.01, api_v='example-api-key'):
        start = arrow.now().floor('day')                                                      # 0:00
        end = arrow.now().shift(days=1).floor('day')                                          # 23:59:59.999
        u='https://api.stormglass.io/v2/solar/point'
        p={'lat': lat_v,'lng': lon_v, 'params': ','.join(['uvIndex']),, 'start': start.to('UTC').timestamp(),  'end': end.to('UTC').timestamp(), }
        h={ 'Authorization': api_v }
        response = requests.get(u, params=p, headers=h)
        json_data = json.dumps(response.json())
        uvi_info=[]                                # this is an array of tuples in the format (height,time,type) as shown by the above idx names
        data_s=WAVE_INF_JSON_T()
        for ele in range(len(json_data['data'])):
            data_s.time = json_data['data'][ele]['time']
            data_s.noaa = float(json_data['data'][0]['uvIndex']['noaa'])
            data_s.meteo = float(json_data['data'][0]['uvIndex']['meteo'])           
            uvi_info.append(data_s)
        lat = json_data['meta']['lat']
        lon = json_data['meta']['lng']
        return uvi_info,lat,lon
        
    def getCompositionStormglass(lat_v=43.38, lon_v=-3.01, api_v='example-api-key'):
        start = arrow.now().floor('day')                                                      # 0:00
        end = arrow.now().shift(days=1).floor('day')                                          # 23:59:59.999
        u='https://api.stormglass.io/v2/bio/point'
        biolist = ['chlorophyll', 'iron', 'soilTemperature100cm', 'soilTemperature40cm', 'soilTemperature10cm', 'soilTemperature', 'soilMoisture100cm', 'soilMoisture40cm', 'soilMoisture10cm', 'soilMoisture', 'nitrate', 'phyto', 'oxygen', 'ph', 'phytoplankton', 'phosphate', 'silicate', 'salinity', ]
        p={'lat': lat_v,'lng': lon_v, 'params': ','.join(biolist), 'start': start.to('UTC').timestamp(),  'end': end.to('UTC').timestamp(), }
        h={ 'Authorization': api_v }
        response = requests.get(u, params=p, headers=h)
        json_data = json.dumps(response.json())
        comp_info=[]                                # this is an array of tuples in the format (height,time,type) as shown by the above idx names
        data_s=COMPOSITION_JSON_T()
        for ele in range(len(json_data['data'])):
            data_s.time = json_data['data'][ele]['time']
            data_s.iron = (float(json_data['data'][ele]['iron']['mercator']),float(json_data['data'][ele]['iron']['sq']))     
            data_s.chlorophyll = (float(json_data['data'][ele]['chlorophyll']['mercator']),float(json_data['data'][ele]['chlorophyll']['sq']))    
            data_s.nitrate = (float(json_data['data'][ele]['nitrate']['mercator']),float(json_data['data'][ele]['nitrate']['sq']))     
            data_s.phyto = (float(json_data['data'][ele]['phyto']['mercator']),float(json_data['data'][ele]['phyto']['sq']))    
            data_s.oxygen = (float(json_data['data'][ele]['oxygen']['mercator']),float(json_data['data'][ele]['oxygen']['sq']))     
            data_s.ph = (float(json_data['data'][ele]['ph']['mercator']),float(json_data['data'][ele]['ph']['sq']))    
            data_s.phosphate = (float(json_data['data'][ele]['phosphate']['mercator']),float(json_data['data'][ele]['phosphate']['sq']))     
            data_s.salinity = (float(json_data['data'][ele]['salinity']['mercator']),float(json_data['data'][ele]['salinity']['sq']))             
            comp_info.append(data_s)
        lat = json_data['meta']['lat']
        lon = json_data['meta']['lng']
        return comp_info,lat,lon    
