import xml.etree.ElementTree as ET
import os
import argparse

# Get KCD path and MSG dir path from args
# Setup argparse
description = 'Python script to generate ROS msg files from can KCD file'
parser = argparse.ArgumentParser(description)
parser.add_argument("kcd_path", help="path to kcd file")
parser.add_argument("msg_path", help="path to msg dir")
args = parser.parse_args()

# Load the KCD file and msg dir
tree = ET.parse(args.kcd_path)
root = tree.getroot()
msg_dir = args.msg_path

# Iterate over each message in the KCD file
for msg in root.findall('Message'):
    # Create a new ROS message file
    file_name = msg.get('name')
    file_path = os.path.join(msg_dir, '' + file_name + '.msg')
    with open(file_path, 'w') as f:
        f.write("# file_name ros message - auto generated from kcd\n\n")

        # Write the message definition to the file
        for sig in msg.findall('Signal'):
            # Get the signal name and data type
            name = sig.get('name').lower()

            # Write the signal definition to the file
            f.write('int32 ' + name + '\n')

    # Print the path of the generated message file
    print(file_path)

