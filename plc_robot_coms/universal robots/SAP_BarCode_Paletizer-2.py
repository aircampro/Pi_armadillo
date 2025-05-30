# example of palletiser robot (not complete)
#
# First, perform the following steps:
#
# Get the code with git clone https://github.com/yomon8/python-hana-connection-sample
# Launch devcontainer
# Installing the required Packages with poetry install
# When you're done here.Please fix the env file. == called .env.sample
#
# Set HDB_HOST to the host name or IP address of the HANA DB.
#
# Since HDB_PORT is the connection port to HANA, it will be the 30215 port for the environment launched with Docker above.
#
# HDB_USER should be set to SAPA4H if you want to select a table visible in the SAP ABAP instance.
#
# The password can be found in the container link.
#
HDB_HOST="192.111.101.3"
HDB_PORT=30215
HDB_USER="SAPA4H"
HDB_PASSWORD="1234"

import sys
import numpy as np
import cv2

# SAP lib
from hana import select_all_as_csv
# Robo Lib
import universal_robot2
   
# find the object (box) index in the SAP list
def find_index( id, list ):
    for j, bn in enumerate(list):
        if bn == id:
            break
    return j

# get box width for the grabber
def get_box_widths( box_id ) :
    sel = "SELECT WIDTH, FROM SAPA4H.USR02 WHERE BOX_TYPE ="+ box_id
    box_width = select_all_as_csv(select_sql=sel)
    return float(box_width)

# look at bar code for corect box type openCV=1 uses openCF asPose=2 uses licensed aspose barcode reader library
openCV=1
asPose=2
def check_box_with_cam( box_id, exp_date, reader=openCV ):
    box_found = 0
    date_found = 0
    
    # take picture using webcam
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture("http://192.168.226.101:8080/video?x.mjpeg")           # android phone webcam on robo
    
    if (cap.isOpened()):

        ret, frame = cap.read()
        if ret == True:
            cv2.imwrite('data/src/barcode.jpg', frame)
            cap.release()
        else:
            print("Camera Not Working.....")
            return False

    # read box type and expiry date from bar code   
    if reader == openCV:    
        bd = cv2.barcode.BarcodeDetector()
        # bd = cv2.barcode.BarcodeDetector('path/to/sr.prototxt', 'path/to/sr.caffemodel')
        retval, decoded_infos, decoded_type, points = bd.detectAndDecode(frame) 
        for decoded_info in decoded_infos:
            print(decoded_info)
            if decoded_info == box_id: 
                box_found = 1   
            elif decoded_info == exp_date:  
                date_found = 1  
            if (box_found == 1) and (date_found == 1):
                break            
    elif reader == asPose:
        from aspose.barcode import barcoderecognition
        reader = barcoderecognition.BarCodeReader('data/src/barcode.jpg', barcoderecognition.DecodeType.AllSupportedTypes)
        recognized_results = reader.read_bar_codes()
        for barcode in recognized_results:
            print(barcode.code_text)
            print(barcode.code_type_name)
            decoded_info = barcode.code_text
            if decoded_info == box_id: 
                box_found = 1   
            elif decoded_info == exp_date:  
                date_found = 1  
            if (box_found == 1) and (date_found == 1):
                break
                
    return (box_found == 1) and (date_found == 1)

# grab the box :: TBD interface to gripper
def grab_box( width, soc ):
    extend_arm_fully(soc)
    # grab_bot( width )
    return True

# move to the given co-ordinate lat, lon location :: TBD check interface with robot
def move_to_loc( lat, lon, soc ):
    move_XYZ_to_dist(soc, x=lat, y=lon, z=0.0)
    return True

# from the total width return the box row posiiton offset
def get_offset_from_width( twid, row ):
    factor = 0.3
    x = twid/2.0 * factor
    y = 5*row
    return (x, y) 

# add 3 pairs to produce a pair (xy co-ordinate) and add the z dimension from the palet layer
# returns a tuple of movement for the robot (x,y,z) sets the angles to 0,0,0
def add_3coords(a, b, c, layer):
    x = a[0] + b[0] + c[0]
    y = a[1] + b[1] + c[1]
    z = 100 - (layer * 10)
    return (x, y, z, 0, 0, 0) 

# place box at coord and rotate 90 degree if needed
# pose is a tuple e.g. (x, y, z, angle_x_radians, angle_y_radians, angle_z_radians) TBD check interface
#
def place_box( coord, rotation, soc ):
    if rotation == 1:
        rot_angle = 90
        # rotate(rot_angle)
        move_in_linear_joint_space(soc, (0,0,0,deg_2_rad(90),0,0), v=0.25, r=0.05, t=0)
    move_in_linear_joint_space(soc, coord, v=0.25, r=0.05, t=0)
    return True

def readCSVLines(line_boxs, line_level_data, line_rotations, line_level_rots):
   # pallet_data[level=0][0][row=0] = "A,A,A,A,E" is bottom level first row
   line =  line_boxs                                 # bottom level first row the csv data is either from a recipe file or SAP ?
   d = line.split(",")
   for zz in d:
       line_level_data.append(ord(zz))
   # pallet_data[level=0][1][row=0] = "0,0,0,0"      # rotations for that bottom level first row
   line =  line_rotations                            # bottom level first row
   d = line.split(",")
   for zz in d:
       line_level_rots.append(zz)

# ============================= main =======================================       

# for odbc driver to system to indicate palet is complete and read the current order to produce
# pip install pyodbc
import pyodbc
# connect to the MIS to read order and complete the order when we have completed
#
if platform.architecture()[0].find("64") == 0:
    cnxn = pyodbc.connect(f'driver=HDBODBC;DSN={dsn};UID={username};PWD={password};SERVERNODE=vhhtzhdpdb.sap.hitachizosen.co.jp:30215') #64bit
else:
    cnxn = pyodbc.connect(f'driver=HDBODBC32;DSN={dsn};UID={username};PWD={password};SERVERNODE=vhhtzhdpdb.sap.hitachizosen.co.jp:30215') #32bit
cursor = cnxn.cursor()
cursor.execute('''SELECT "ORDER_STR"
                  FROM "_SYS_ORD"."s4.sd/CA_SD_WORKING_ORDER"
                  WHERE "BAY_NO" = '12E4' and "FREEZER_NO" = 'XRD1';
               ''')
order_str = cursor[0]                                         # first string returned from query is order number for current job 
# create query to use when completing the order (feedback)
cur=("INSERT INTO \"_SYS_ORD\".\"s4.sd/CA_SD_COMPLETE_ORDER\" VALUES ("+order_str+");")

# choose the data related to the order from the SAP system
#order_str = '384XSD2321'
sel = "SELECT QTY, FROM SAPA4H.USR02 WHERE ORDER = "+ order_str
qty = select_all_as_csv(select_sql=sel)             # to print select_all_as_csv(select_sql=sel.output=sys.stdout)
sel = "SELECT BOXS, EXPIRY_DATES FROM SAPA4H.USR02 WHERE ORDER = "+ order_str
boxs = select_all_as_csv(select_sql=sel)
sel = "SELECT EXPIRY_DATES, FROM SAPA4H.USR02 WHERE ORDER = "+ order_str
exp_dates = select_all_as_csv(select_sql=sel)       # df = select_all_as_df(select_sql=sql)

# write the data to a csv file for storage or debug
path = 'data/src/sap_hana_output.csv'
f = open(path)
sel = "SELECT QTY, BOXS, EXPIRY_DATES FROM SAPA4H.USR02 WHERE ORDER = " +order_str
select_all_as_csv(select_sql=sel.output=f)
f.close()

# make lists from SAP info in csv unmatching lengths are data errors
if (len(qty) == len(boxs)) and (len(qty) == len(exp_dates)):
    qty = qty.split(",")
    boxs = boxs.split(",")
    exp_dates = exp_dates.split(",")
else:
    print("invalid SAP data returned, inconsistent lengths of data (box types, quantity and expiry date)")
    sys.exit(-1)

# from the order make a unique integer for parsing, this might be more complex with 
# many boxs you could also read this data from a relational table in SAP
#
recipe_id = 0
for i, q in enumerate(qty):
    if boxs[i] == "A":
        recipe_id = recipe_id | int(q)
    elif boxs[i] == "B":
        recipe_id = recipe_id | int(q)<<8    
    elif boxs[i] == "C":
        recipe_id = recipe_id | int(q)<<16
    elif boxs[i] == "D":
        recipe_id = recipe_id | int(q)<<24
    elif boxs[i] == "G":
        recipe_id = recipe_id | int(q)<<32
    elif boxs[i] == "H":
        recipe_id = recipe_id | int(q)<<40
    elif boxs[i] == "I":
        recipe_id = recipe_id | int(q)<<48
    elif boxs[i] == "J":
        recipe_id = recipe_id | int(q)<<56
    elif boxs[i] == "K":
        recipe_id = recipe_id | int(q)<<64
        
# now choose the pallet config (levels) boxs with (rotation) 0=none 1=90 degree
level=0                                                 # start at bottom level (stack)
row=0                                                   # start at first line (row)

# make 3d arrays of the pallet set-up for each combination of boxes defined 
# this is the optimal set for a specified selection of boxes
# the data is followed by possible rotations for each box  
# this could come from csv files produced for each recipe, hardcoded like here, or from SAP for that recipe
# its hard coded to easy show you the format
#              
if recipe_id == (8 | 10<<8 | 10<<16):                  # 8 A, 10 B, 10 C

   # pallet_data[level=0][0][row=0] = "A,A,A,A,E" is bottom level first row
   # pallet_data[level=0][1][row=0] = "0,0,0,0"                 # rotations for that bottom level first row
   line_boxs =  "A,A,A,A,E"                                     # bottom level first row the csv data is either from a recipe file or SAP ?
   line_rotations =  "0,0,0,0"                                  # bottom level first row
   line1_level1_rots = []
   line1_level1_data = []
   readCSVLines(line_boxs, line1_level1_data, line_rotations, line1_level1_rots)
   # pallet_data[level=0][1][row=0] = "0,0,0,0"                   # rotations for that bottom level first row
 
   #pallet_data[level=0][0][row=1] = "B,B,C,C,C,E"  
   line_boxs =  "B,B,C,C,C,E"                                  
   # pallet_data[level=0][1][row=1] = "0,0,0,1,1"   
   line_rotations =  "0,0,0,1,1"                              
   line2_level1_rots = []
   line2_level1_data = []
   readCSVLines(line_boxs, line2_level1_data, line_rotations, line2_level1_rots)
   
   #pallet_data[level=0][0][row=2] = "B,B,B,C,C,L" 
   line_boxs =  "B,B,B,C,C,L"                                 
   #pallet_data[level=0][1][row=2] = "1,1,1,1,1"   
   line_rotations =  "1,1,1,1,1"                                 
   line3_level1_rots = []
   line3_level1_data = []
   readCSVLines(line_boxs, line3_level1_data, line_rotations, line3_level1_rots)
   
   #   pallet_data[level=1][0][row=0] = "A,A,A,A,E" 
   line_boxs =  "A,A,A,A,E"                                 
   #    pallet_data[level=1][1][row=0] = "0,0,0,0"   
   line_rotations =   "0,0,0,0"                                  
   line1_level2_rots = []
   line1_level2_data = []
   readCSVLines(line_boxs, line1_level2_data, line_rotations, line1_level2_rots)
   
   # pallet_data[level=1][0][row=1] = "B,B,C,C,C,E"
   line_boxs =  "B,B,C,C,C,E"                                 
   # pallet_data[level=1][1][row=1] = "0,0,0,1,1"    
   line_rotations =   "0,0,0,1,1"                                   
   line2_level2_rots = []
   line2_level2_data = []
   readCSVLines(line_boxs, line2_level2_data, line_rotations, line2_level2_rots)  

   #pallet_data[level=1][0][row=2] = "B,B,B,C,C,F" 
   line_boxs =  "B,B,B,C,C,F"                                 
   #pallet_data[level=1][1][row=2] = "1,1,1,1,1"    
   line_rotations =    "1,1,1,1,1"                                   
   line3_level2_rots = []
   line3_level2_data = []
   readCSVLines(line_boxs, line3_level2_data, line_rotations, line3_level2_rots)
   
if recipe_id == (7 | 10<<8 | 10<<16):                    # 7 A, 10 B, 10 C

   # pallet_data[level=0][0][row=0] = "A,A,A,A,E" is bottom level first row
   # pallet_data[level=0][1][row=0] = "0,0,0,0"                 # rotations for that bottom level first row
   line_boxs =  "A,A,A,A,E"                                     # bottom level first row the csv data is either from a recipe file or SAP ?
   line_rotations =  "0,0,0,0"                                  # bottom level first row
   line1_level1_rots = []
   line1_level1_data = []
   readCSVLines(line_boxs, line1_level1_data, line_rotations, line1_level1_rots)
   # pallet_data[level=0][1][row=0] = "0,0,0,0"                   # rotations for that bottom level first row
 
   #pallet_data[level=0][0][row=1] = "B,B,C,C,C,E"  
   line_boxs =  "B,B,C,C,C,E"                                  
   # pallet_data[level=0][1][row=1] = "0,0,0,1,1"   
   line_rotations =  "0,0,0,1,1"                              
   line2_level1_rots = []
   line2_level1_data = []
   readCSVLines(line_boxs, line2_level1_data, line_rotations, line2_level1_rots)
   
   #pallet_data[level=0][0][row=2] = "B,B,B,C,C,L" 
   line_boxs =  "B,B,B,C,C,L"                                 
   #pallet_data[level=0][1][row=2] = "1,1,1,1,1"   
   line_rotations =  "1,1,1,1,1"                                 
   line3_level1_rots = []
   line3_level1_data = []
   readCSVLines(line_boxs, line3_level1_data, line_rotations, line3_level1_rots)
   
   #   pallet_data[level=1][0][row=0] = "A,A,A,A,E" 
   line_boxs =  "A,A,A,A,E"                                 
   #    pallet_data[level=1][1][row=0] = "0,0,0,0"   
   line_rotations =   "0,0,0,0"                                  
   line1_level2_rots = []
   line1_level2_data = []
   readCSVLines(line_boxs, line1_level2_data, line_rotations, line1_level2_rots)
   
   # pallet_data[level=1][0][row=1] = "B,B,C,C,C,E"
   line_boxs =  "B,B,C,C,C,E"                                 
   # pallet_data[level=1][1][row=1] = "0,0,0,1,1"    
   line_rotations =   "0,0,0,1,1"                                   
   line2_level2_rots = []
   line2_level2_data = []
   readCSVLines(line_boxs, line2_level2_data, line_rotations, line2_level2_rots)  

   #pallet_data[level=1][0][row=2] = "B,B,B,C,C,F" 
   line_boxs =  "B,B,B,C,C,F"                                 
   #pallet_data[level=1][1][row=2] = "1,1,1,1,1"    
   line_rotations =    "1,1,1,1,1"                                   
   line3_level2_rots = []
   line3_level2_data = []
   readCSVLines(line_boxs, line3_level2_data, line_rotations, line3_level2_rots)

# make recipe array(s) from our data set chosen from the given recipe above it comprises of 3 rows(box and rotation) and 2 stack levels
palet_data_lev1 = np.array([[line1_level1_data,
                   line1_level1_rots],

                  [line2_level1_data,
                   line2_level1_rots],
                   
                  [line3_level1_data,
                   line3_level1_rots]])
palet_data_lev2 = np.array([[line1_level2_data,
                   line1_level2_rots],

                  [line2_level2_data,
                   line2_level2_rots],
                   
                  [line3_level2_data,
                   line3_level2_rots]])

# build our pallet from each layer array defined above
pallet_levls = [ palet_data_lev1, palet_data_lev2 ]
                   
# move through the scan locations where the boxs are stored pick box and move it to the pallet location
scan_loc = [ (23,109), (32.109), (100,230) ]
palet_loc = (200,79)
# positioning array this has to be calculated - the data is dummy (can we use presets in the robot) ?
palet_pos_offset = palet_loc + (20,20)
palet_row_offset = (5,0)
palet_wid = 0

# level is the stack row is the horizontal line box_no is the number in the sequence place is set when the box was found and picked up
# for us to then place it.
#
# we have a manual entry here if we want to start the sequence mid way (return from an error)
#
if len(sys.argv) >= 13:                             # passed a manual pickup point to start from via the arguments
    level = int(sys.argv[1])
    row = int(sys.argv[2])
    box_no = int(sys.argv[3])
    box_a_cnt = int(sys.argv[4])                    # re-start the box counters with what we have already in the pallet
    box_b_cnt = int(sys.argv[5])
    box_c_cnt = int(sys.argv[6])
    box_d_cnt = int(sys.argv[7])
    box_g_cnt = int(sys.argv[8])
    box_h_cnt = int(sys.argv[9])
    box_i_cnt = int(sys.argv[10])
    box_j_cnt = int(sys.argv[11])    
    box_k_cnt = int(sys.argv[12]) 
    if len(sys.argv) >= 14:    
        place = int(sys.argv[13])                   # start palacing rather than picking if set to non-zero
else:                                               # start at the begining
    level = 0
    row = 0
    box_no = 0
    box_a_cnt = 0
    box_b_cnt = 0
    box_c_cnt = 0
    box_d_cnt = 0
    box_g_cnt = 0
    box_h_cnt = 0
    box_i_cnt = 0
    box_j_cnt = 0    
    box_k_cnt = 0      
    # default the place flag until the box was identified and picked up    
    place = 0

# data organisation layout for 2d array(s) representing each row
box = 0                  # first the boxes
rot = 1                  # next the rotations

# connect to UR robot
soc = robot_connect("192.168.250.112", 30004)

# perfrom the palletization operation until we reach the box label F = Finish
exit_scan = 0   
while True:

    # --------- scan for box and pick it --------------
    for s in scan_loc:
        if exit_scan == 1:
            break
        move_to_loc(s[0],s[1],soc)                                       # advance to each location (bay) where boxs are stored  
        sel_lvl = pallet_levls[level]                                    # select the correct numpy array for the stack level
        box_match = false
        box_to_find = sel_lvl[row][box][box_no]                          # get next box requirement        
        if box_to_find == ord("E"):                                      # denotes new row increment the row index and reset the total width on that row
            row += 1
            palet_wid = 0
        elif box_to_find == ord("L"):                                    # denotes next level
            level += 1
            if (level >= len(pallet_levls)) :
                print("exceeded maximum stacks ", level)
                sys.exit(-1)
        elif box_to_find == ord("Z"):                                    # denotes empty position skip and increment to next
            box_no += 1
        elif box_to_find == ord("F"):                                    # denotes pallet complete
            print("palletising complete.....")
            print("box A : ",box_a_cnt,"box B : ",box_b_cnt,"box C : ",box_c_cnt,"box D : ",box_d_cnt)
            print("box G : ",box_g_cnt,"box H : ",box_h_cnt,"box I : ",box_i_cnt,"box J : ",box_j_cnt,"box K : ",box_k_cnt)
            order_complete = True                                        # check if the quantity calculated matches the quantities in the order specified
            for i, q in enumerate(qty):
                if boxs[i] == "A":
                    order_complete &= box_a_cnt == int(q)
                if boxs[i] == "B":
                    order_complete &= box_b_cnt == int(q)   
                if boxs[i] == "C":
                    order_complete &= box_c_cnt == int(q)
                if boxs[i] == "D":
                    order_complete &= box_d_cnt == int(q)
                if boxs[i] == "G":
                    order_complete &= box_g_cnt == int(q)
                if boxs[i] == "H":
                    order_complete &= box_h_cnt == int(q)
                if boxs[i] == "I":
                    order_complete &= box_i_cnt == int(q)
                if boxs[i] == "J":
                    order_complete &= box_g_cnt == int(q)
                if boxs[i] == "K":
                    order_complete &= box_k_cnt == int(q)
                print("Order Complete ", order_complete)
            if order_complete == True:
                cursor.execute(cur)                                          # coplete the order in MIS odbc database
                cursor.commit()                 
            # disconnect UR robot
            robot_disconnect(soc) 
            # disconnect from odbc
            # cursor and connection close
            cursor.close()
            cnxn.close()
            # set pointers to nulls
            cursor = None
            cnxn = None
            sys.exit(0)
        else:                                                                # should be a valid box
            while True:                                                      # keep grabbing box until done
                box_match = check_box_with_cam(chr(box_to_find), exp_dates[find_index[chr(box_to_find), boxs]])   # look for it in stock
                if box_match == True:
                    box_width = get_box_widths(chr(box_to_find))             # get the box types width from the SAP data
                    if (grab_box(box_width, soc) == True):                   # succesfully grabbed the box
                        place = 1                                            # set to place it
                        palet_wid += box_width
                        exit_scan = 1
                        break                                                # exit and place it
                    else:
                        print("Failed to grab box")
                        # sys.exit(-1)
                else:
                    print("wrong box or wrong date @loc ",s[0]," ",s[1])

    # ------------ move to palet and place box --------------                    
    while (place == 1) :                                                 # now place the box loop on error
        move_to_loc(palet_loc[0], palet_loc[1], soc)                     # move to pallet defined location
        if (place_box(add_3coords(palet_pos_offset+(palet_row_offset*row)+get_offset_from_width(palet_wid), level), sel_lvl[row][rot][box_no], soc) == True):
            box_no += 1
            place = 0 
            # increment the count of boxes for the specified box placed
            if box_to_find == ord("A"):                                    
                box_a_cnt += 1        
            elif box_to_find == ord("B"):                                    
                box_b_cnt += 1 
            elif box_to_find == ord("C"):                                    
                box_c_cnt += 1 
            elif box_to_find == ord("D"):
                box_d_cnt += 1 
            elif box_to_find == ord("G"):
                box_g_cnt += 1 
            elif box_to_find == ord("H"):
                box_h_cnt += 1 
            elif box_to_find == ord("I"):
                box_i_cnt += 1 
            elif box_to_find == ord("J"):
                box_j_cnt += 1 
            elif box_to_find == ord("K"):
                box_k_cnt += 1 
# disconnect UR robot
robot_disconnect(soc)   