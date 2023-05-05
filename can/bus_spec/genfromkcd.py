import untangle
import os
import argparse
import fileinput

kcd = "david.kcd"
msg_dir = "../../ros/catkin_ws/src/motor_bridge/msg"
cmake = "../../ros/catkin_ws/src/motor_bridge/CMakeLists.txt"

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
#include <ros/ros.h>
"""

c_template = """
/*
 * {0}
 * Can message generation helpers
 * Auto generated from KCD file
 */
#include "{1}"
#include "david.h"

"""

msgs = {}

def camel_to_snake(s):
    if s.isupper():
        return s.lower()
    return ''.join(['_'+i.lower() if i.isupper()
            else i for i in s]).lstrip('_')

# Create Messages
# Combined system message
system_msg_path = os.path.join(msg_dir, 'System.msg')
with open(system_msg_path, 'w') as s:
    s.write("# System message - auto generated from kcd\n\n")
    # Iterate over each message in the KCD file
    for msg in tree.NetworkDefinition.Bus.Message:
        # Create a new ROS message file
        msg_name = msg['name']
        msg_path = os.path.join(msg_dir, '' + msg_name + '.msg')

        names = []
        with open(msg_path, 'w') as m:
            m.write("# " + msg_name +
                    " ros message - auto generated from kcd\n\n")

            # Write the message definition to the file
            for sig in msg.Signal:
                # Write the signal definition to the file
                name = camel_to_snake(sig['name'])
                names.append(name)
                size = sig['length']
                m.write('int' + size + ' ' + name + '\n')

        msgs[msg_name] = names

        # Add new message to combined message
        s.write(msg_name + " " + camel_to_snake(msg_name) + '\n')

with open(can_c, 'w') as c, open(can_h, 'w') as h:
    # Add stuff to c and h that always needs to be there
    h.write(h_template.format(can_h))
    c.write(c_template.format(can_c, can_h))

    for msg in msgs.keys():
        h.write("#include <motor_bridge/" + msg + ".h>\n")

    h.write('\n')
    for msg, names in msgs.items():
        # Add functions to can_h and can_c
        func = ("int pack_msg(const motor_bridge::"
                + msg + "& msg, uint8_t* buff)")
        h.write(func + ";\n")
        c.write(func + " {\n")
        c.write("    david_" + camel_to_snake(msg) + "_t t = {\n")
        for name in names:
            c.write("        ." + name + " = msg." + name + ",\n")

        c.write("    };\n\n")
        c.write("    david_" + camel_to_snake(msg) + "_pack(buff, &t, 8);\n")
        c.write("    return DAVID_" +
                camel_to_snake(msg).upper() + "_FRAME_ID;\n")
        c.write("}\n\n")
    
    h.write('\n')
    for msg, names in msgs.items():
        func = "std::ostream& operator<<(std::ostream& s, const motor_bridge::" + msg + "& msg)"
        h.write(func + ";\n")
        c.write(func + " {\n")
        c.write("    s << \"" + msg  + "\" << \" - \";\n")
        for name in names:
            c.write("    s << \"" + name + ": \" << msg." + name + " << \", \";\n")
        c.write("    s << std::endl;\n")
        c.write("    return s;\n")
        c.write("}\n\n")

tgt = "add_message_files("
lno = 0;
i1 = 0;
i2 = 0;
data=[]
with open(cmake, 'r') as f:
    data = f.readlines()

cline = data[lno].strip()
while cline != tgt:
    lno += 1
    cline = data[lno].strip()
i1 = lno + 3

while cline != ")":
    lno += 1
    cline = data[lno].strip()
i2 = lno

new_msgs = []
new_msgs.append('System.msg\n')
for key in msgs.keys():
    new_msgs.append('    ' + key + '.msg\n')

data[i1:i2] = new_msgs

with open(cmake, 'w') as f:
    for line in data:
        f.write(line)
