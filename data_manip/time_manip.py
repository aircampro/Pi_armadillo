# python time manipulation library
#
from datetime import datetime, timedelta, timezone

# a function to show japan date and time
def show_time_japan(self):
    JST = timezone(timedelta(hours=+9), 'JST')
    dt_now = datetime.now(JST)
    jap_date_time = dt_now.strftime("%Y%m%d_%H%M%S")
    return jap_date_time
	
def show_time_ny(self):
    JST = timezone(timedelta(hours=-5), 'NYT')
    dt_now = datetime.now(NYT)
    ny_date_time = dt_now.strftime("%Y%m%d_%H%M%S")
    return ny_date_time
	
	