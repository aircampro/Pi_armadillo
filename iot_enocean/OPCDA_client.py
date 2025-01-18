# example of OPC DA client
#
import OpenOPC
import pywintypes

def main():
    # Create OPC DA client object
    pywintypes.datetime = pywintypes.TimeType
    opc = OpenOPC.client()

    # Search for OPC DA servers
    servers = opc.servers()
    print(servers)

    # Connection to OPC DA Server
    opc.connect('Matrikon.OPC.Simulation.1')
    aliases = opc.list()

    # Use list() to explore available items
    groups = opc.list('Simulation Items.Random')
    print(groups)

    # Read() method return a tuple with value, quality and timestamp of that tag
    r = opc.read(‘Random.Int32’)
    print(r)
    
    # Locate desired variable and apply write method to write value
    w = opc.write(('Bucket Brigade.Int4',5))
    print(w)
    
    # Define tag group list and write value tuple 
    tag_group = ['Bucket Brigade.Int4','Bucket Brigade.Int2']
    tag_group_write = [('Bucket Brigade.Int4',10),('Bucket Brigade.Int2',8)]
    print(tag_group_write)
    
    # Pass on tag group to read and write method
    opc.write(tag_group_write)
    r = opc.read(tag_group)
    print(r)
    
if __name__ == '__main__':
    main()