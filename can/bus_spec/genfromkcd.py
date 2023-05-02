import untangle
import os
import argparse

kcd = "david.kcd"
msg_dir = "../../ros/catkin_ws/src/motor_bridge/msg"

# Load the KCD file and msg dir
tree = untangle.parse(kcd)

# Can file names
can_c = "canshit.cpp"
can_h = "canshit.hpp"

h_template = """
/*
 * {0}
 * Can message generation helpers
 * Auto generated from KCD file
 */

#include <iostream>

"""

c_template = """
/*
 * {0}
 * Can message generation helpers
 * Auto generated from KCD file
 */
#include "{1}"

void to_bytes(int n, uint8_t* buff, int start) {{
    if (start + sizeof(n) >= 8)
        throw string("Not enough space in can frame");
    memcpy(buff + start, &n, sizeof(n));
}}

"""

# Create combined system message
file_path = os.path.join(msg_dir, 'System.msg')
with open(file_path, 'w') as s, open(can_c, 'w') as c, open(can_h, 'w') as h:
    # Add stuff to c and h that always needs to be there
    h.write(h_template.format(can_h))
    c.write(c_template.format(can_c, can_h))

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
        func = ("void pack_msg(motor_bridge::"
                + msg_name + " msg, uint8_t* buff)")
        h.write(func + ";\n")
        c.write(func + " {\n")

        with open(file_path, 'w') as f:
            f.write("# " + msg_name +
                    " ros message - auto generated from kcd\n\n")
    
            # Write the message definition to the file
            for sig in msg.Signal:
                # Get the signal name and data type
                # Change name from camel to snake case
                name = ''.join(['_'+i.lower() if i.isupper()
                    else i for i in sig['name']]).lstrip('_')
                size = sig['length']
                # Write the signal definition to the file
                f.write('int' + size + ' ' + name + '\n')

                # Write into c and h files
                offset = sig['offset']
                c.write("   to_bytes(msg." + name +
                        ", buff, " + offset + ");\n")
        c.write("}\n\n")


