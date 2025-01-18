# opcua client example
#
from opcua import ua,Client
import socket
from opcua.ua import NodeId

def main():
    #Find out the connection string of OPC UA Server and initiate client object, replace {} with OPC UA Server hostname
    url =”opc.tcp://{}.localdomain:53530/OPCUA/SimulationServer”.format(socket.gethostname())
    client = Client(url)

    #Connection to the OPC UA Server
    client.connect()

    # Call get_root_node method to locate node at the top of hierachy
    root_node = client.get_root_node()

    # Use get_children to drill down the hierachy, get_browse_name method allows you to view node browse name instead of nodeID
    [ x.get_browse_name() for x in root_node.get_children()]

    # Locate object node where our variables are stored
    objects = root_node.get_children()[0]

    # Or you can use get_objects_node to directly locate objects node
    objects = client.get_objects_node()

    #Browse for the children nodes of the objects node
    [ x.get_browse_name() for x in objects.get_children()]

    # Use get_children method to explore children of the object node
    # Use index to choose children node of desire 
    sim_data_node = objects.get_children()[2]

    #Apply get_browse_name to get node name instead of node id
    [ {"NodeID":x ,"Node Name": x.get_browse_name()} for x in sim_data_node.get_children()]

    # You may specify NodeId using NodeId object or string
    # The two method below reaches the same node
    client.get_node(NodeId(1004,3)).get_browse_name()

    # Get to specific node by specifying node index using get_node method
    client.get_node('ns=3;i=1004').get_browse_name()

    #Index desired variable and apply get_value method to read value
    sim_data_node_sin = client.get_node('ns=3;i=1004')
    sim_data_node_sin.get_value()

    # Index desired variable and apply set_value method to write value
    sim_data_node_test =  client.get_node('ns=3;i=1007')
    sim_data_node_test.set_value(4.0)
    
if __name__ == '__main__':
    main()
