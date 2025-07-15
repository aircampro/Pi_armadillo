#!/usr/bin/env python3
# codint: utf-8
# Example of distance probe it reads the probe and put data into sqllite database table and show a trended values plot
# HC-SR04 range 2-180cm - Sensor module that uses the reflection time of supersound for non-contact ranging
#
import RPi.GPIO as GPIO
import time
import atexit
import sqlite3 as sqlite
import datetime
import GraphUtils as Gu

# GPIO PIN number
# it uses 2 gpio pins the trigger and echo
#
DEF_TRIG_GPIO = 17
DEF_ECHO_GPIO = 27

# Options
NOW = "now"
HOUR = "hour"
DAY = "day"
OPTIONS = [NOW, HOUR, DAY]

# Message
USONIC_NOW_FMT = "Current distance is {0}[cm]."
USONIC_LOG_FMT = "Distance log in {0}."

# default files for trend display
USONIC_GRAPH_TEMP = "./image/usonic_temp.png"
    
# class for ultrasonic distance sensor
class USonic(TRIG_GPIO = DEF_TRIG_GPIO, ECHO_GPIO = DEF_ECHO_GPIO ):

    def __init__(self):
        super(USonic, self).__init__("usonic", 1)
        self.tweet = tweet
        # Specify pin number as General Puroise Pin number
        GPIO.setmode(GPIO.BCM)
        # Set usonic pin
        GPIO.setup(TRIG_GPIO, GPIO.OUT)
        GPIO.setup(ECHO_GPIO, GPIO.IN)
        # Initialize pin
        self.__init_usonic()
        # Register cleanup handler at exit
        atexit.register(self.__cleanup)

    def __cleanup(self):
        GPIO.cleanup(TRIG_GPIO)
        GPIO.cleanup(ECHO_GPIO)

    def __init_usonic(self):
        GPIO.output(TRIG_GPIO, GPIO.LOW)
        time.sleep(0.3)

    # Concrete methods of super class
    def operate(self, args):
        option = args[0]
        if option == NOW:
            self.__usonic_now()
        else:
            self.__usonic_log(span=option)

    def check_args(self, args):
        return args[0] in OPTIONS

    """
    Usonic current data.
    """
    def __usonic_now(self):
        distance = self.measure_distance()
        message = USONIC_NOW_FMT.format(str(distance))
        print(message)

    def __usonic_log(self, span=HOUR):
        now = datetime.datetime.now()
        if span == DAY:
            from_time = now - datetime.timedelta(days=1)
            axis_span = 120
        else:
            from_time = now - datetime.timedelta(hours=1)
            axis_span = 10
        db = USonicDB()
        log = db.select_between(from_time, now)
        Gu.create_usonic_graph(log, USONIC_GRAPH_TEMP, axis_span=axis_span)
        message = USONIC_LOG_FMT.format(span)
        print(message)

    def __fetch_log(self, from_time, to_time):
        # Read log from db file
        return self.db.select_between(from_time, to_time)

        
    """
    Measure distance with usonic sensor
    """
    def measure_distance(self):
        # Send pulse
        GPIO.output(TRIG_GPIO, True)
        time.sleep(0.00001)
        GPIO.output(TRIG_GPIO, False)
        # Wait while ECHO pin input
        while GPIO.input(ECHO_GPIO) == 0:
            signaloff = time.time()
        while GPIO.input(ECHO_GPIO) == 1:
            signalon = time.time()
        return USonic.__time_to_centm(signalon - signaloff)

    """
    Convert usonic time to sentimetres.
    @param sec : usonic response time
    """
    @classmethod
    def __time_to_centm(cls, sec):
        return sec * 17000

# default name for database files
DB_FILE = "usonic/usonic.db"

# SQL queries
CREATE_TABLE_SQL = u"""
    CREATE TABLE IF NOT EXISTS USONIC_LOG (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        date TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
        distance REAL
    );
    """
INSERT_SQL = u"""
    INSERT INTO USONIC_LOG (distance) values (?);
"""
SELECT_BETWEEN_SQL = u"""
    SELECT * FROM USONIC_LOG 
    WHERE date BETWEEN ? AND ?;
"""
SELECT_SQL = u"""
    SELECT * FROM USONIC_LOG;
"""

# class for interfacing with sqllite database
class USonicDB:
    
    def __init__(self, db_file=DB_FILE):
        self.db_file = db_file
        try:
            self.conn = sqlite.connect(db_file)
            print("Connection established")
            self.__create_table()
        except sqlite.Error, e:
            print ("Sqlite connect error.")
        # Register cleanup handler at exit
        atexit.register(self.__cleanup)

    def __cleanup(self):
        try:
            self.conn.close()
            print("Connection closed.")
        except sqlite.Error, e:
            print("Sqlite connect error.")

    def __create_table(self):
        try:
            self.conn.execute(CREATE_TABLE_SQL)
            self.conn.commit()
        except sqlite.Error, e:
            print("Create table failed.")
            print(e)

    def insert_log(self, distance):
        try:
            self.conn.execute(INSERT_SQL, [distance])
            self.conn.commit()
        except sqlite.Error, e:
            print("Insert error.")

    def select_between(self, start, end):
        try:
            cursor = self.conn.cursor()
            cursor.execute(SELECT_BETWEEN_SQL, [start, end])
            return [USonicData(row[0], datetime.datetime.strptime(row[1], "%Y-%m-%d %H:%M:%S"), row[2]) for row in cursor]
        except sqlite.Error, e:
            print("Select error.")
            print(e)
            return []

    def select(self):
        try:
            cursor = self.conn.cursor()
            cursor.execute(SELECT_SQL)
            return [USonicData(row[0], datetime.datetime.strptime(row[1], "%Y-%m-%d %H:%M:%S"), row[2]) for row in cursor]
        except sqlite.Error, e:
            print("Select error.")
            print(e)
            print []

class USonicData:
    
    def __init__(self, id, date, distance):
        self.id = id
        self.date = date
        self.distance = distance

    def get_id(self):
        return self.id

    def get_date(self):
        return self.date

    def get_distance(self):
        return self.distance

"""
On launching this python on main, get distance data and insert log.
"""
if __name__ == "__main__":
    db = USonicDB()
    usonic = USonic() 
    try:
        while True:
            distance = usonic.measure_distance()
            db.insert_log(distance)
    except( KeyboardInterrupt, SystemExit):
        print("distance sensor measurement interrupted")
    
