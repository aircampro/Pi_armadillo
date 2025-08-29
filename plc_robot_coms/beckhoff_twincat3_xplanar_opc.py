# ------------- supported robots by RoboDK library ---------------------
# full info is shown here https://robodk.com/doc/en/Basic-Guide.html#Guide
# to install it :- pip install robodk
#
# ABB which support RAPID
# Annin Robotics
# AUBO
# Automata
# Borunte
# Comau
# Denso
# Dobot
# Doosan
# Elephant Robotics
# Elite Robots
# Epson
# FAIRINO
# Fanuc
# Han’s Robot / Huayan Robotics
# Hanwha
# IGUS
# JAKA
# Kawasaki
# KEBA
# Kinova
# KUKA KRC
# KUKA IIWA
# Mecademic
# Mitsubishi
# Niryo
# Productive Robotics
# Siemens
# Staubli
# Techman/Omron
# uFactory xArm
# Universal Robots
# Wlkata
# Yaskawa Motoman
#
# other items include planars and cameras
#
# ---------------------------------------------------------------------------------
#
# Example shows how to send the pitch roll and yaw from OPCUA server to a Beckhoff XPLANAR
# Beckhoff TwinCAT3 Play with Xplanar And TF5430_Part1
#
from robodk import robolink        # RoboDK API
from robodk import robomath        # Robot toolbox
from opcua import Client,ua        # For OPC Client connection

if __name__ == "__main__":
    # Start the RoboDK API:
    RDK = robolink.Robolink()

    # define the name for the xplanar in RoboDK
    XPLANAR=’xplanar1′
    # define the OPC server endpoint 
    ENDPOINT=”opc.tcp://DESKTOP-7EN1MN2:4840″
    # define the node point containing the item which has the pose data
    NODES=’ns=4;s=GVL2RoboDK.xplanar1.pos.ActPos’
    NODER=’ns=4;s=GVL2RoboDK.robo1.pos.ActPos’
    # connect client to the endpoint
    client=Client(ENDPOINT)

    # Select the XPLANAR
    xplanar = RDK.Item(XPLANAR)
    if xplanar.Valid():
        print(‘Item selected: ‘ + xplanar.Name())
        print(‘Item posistion: ‘ + repr(xplanar.Pose()))
    else:
        raise Exception('No xplanar selected or available')
    # Select a robot (popup is displayed if more than one robot is available)
    robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
    # robot = RDK.Item('Fanuc LR Mate 200iD', ITEM_TYPE_ROBOT) or hard code it like this
    if not robot.Valid():
        raise Exception('No robot selected or available')
    else:
        print(‘Item selected: ‘ + robot.Name())
        print(‘Item posistion: ‘ + repr(robot.Pose()))    
    # get the current position of the TCP with respect to the reference frame:
    # (4x4 matrix representing position and orientation)
    target_ref = robot.Pose()
    pos_ref = target_ref.Pos()
    print(Pose_2_TxyzRxyz(target_ref))
    # move the robot joints to the first point:
    robot.MoveJ(target_ref)  
 
    # connect OPC Client to the server
    client.connect()
    print(‘Connected to OPCUA server..’)
    # get the nodes from the opc server
    NodeFromServer=client.get_node(NODES)
    NodeRoFromServer=client.get_node(NODER)
    client.load_type_definitions()

    # read the node values and set the xplanar and robot to those
    RUNNG = True
    try:
        while RUNNG == True:
            Actpos=NodeFromServer.get_value()
            CurrentPos=xyzrpw_2_pose([Actpos.x,Actpos.y,Actpos.z,Actpos.a,Actpos.b,Actpos.c])
            xplanar.setPoseAbs(CurrentPos)
            Ropos=NodeRoFromServer.get_value()
            target_ref=xyzrpw_2_pose([Ropos.x,Ropos.y,Ropos.z,Ropos.a,Ropos.b,Ropos.c])
            robot.MoveJ(target_ref) 
    except KeyboardInterrupt:
        RUNNG = False
    # disconnect from OPC server
    client.disconnect()