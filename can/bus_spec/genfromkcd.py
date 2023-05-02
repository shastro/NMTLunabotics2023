import untangle
import os
import argparse

# Get KCD path and MSG dir path from args
# Setup argparse
description = 'Python script to generate ROS msg files from can KCD file'
parser = argparse.ArgumentParser(description)
parser.add_argument("--kcd", help="path to kcd file", type=str, required=True)
parser.add_argument("--msg", help="path to msg dir", type=str, required=True)
args = parser.parse_args()

# Load the KCD file and msg dir
tree = untangle.parse(args.kcd)
msg_dir = args.msg

# Can file names
can_c = "canshit.cpp"
can_h = "canshit.hpp"

# Create combined system message
file_path = os.path.join(msg_dir, 'System.msg')
with open(file_path, 'w') as s, open(can_c, 'w') as c, open(can_h, 'w') as h:
    s.write("# System message - auto generated from kcd\n\n")
    # Iterate over each message in the KCD file
    for msg in tree.NetworkDefinition.Bus.Message:
        # Create a new ROS message file
        msg_name = msg['name']
        # Change name from camel to snake case
        msg_name_snake = ''.join(['_'+i.lower() if i.isupper()
            else i for i in msg_name]).lstrip('_')
        # Add each message to system message
        s.write('' + msg_name + ' ' + msg_name_snake + '\n')
        file_path = os.path.join(msg_dir, '' + msg_name + '.msg')
        print(file_path)

        # Add functions to can_h and can_c
        func = "void send_msg( motor_bridge::" + msg_name + " msg )"
        h.write(func + ";\n")
        c.write(func + " {\n")

        with open(file_path, 'w') as f:
            f.write("# " + file_name + " ros message - auto generated from kcd\n\n")
    
            # Write the message definition to the file
            for sig in msg.Signal:
                # Get the signal name and data type
                # Change name from camel to snake case
                name = ''.join(['_'+i.lower() if i.isupper()
                    else i for i in sig['name']]).lstrip('_')
                size = sig['length']
        
                # Write the signal definition to the file
                f.write('int' + size + ' ' + name + '\n')


