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
ards_h = "unpackers.hpp"
lim = "limits.hpp"

template = """
/*
 * {0}
 * Can message generation helpers
 * Auto generated from KCD file
 */

"""

packet = """
struct CANPacket {
    uint32_t len;
    uint32_t id;
    unsigned char buff[8];
};

"""


msgs = []

def camel_to_snake(s):
    if s.isupper():
        return s.lower()
    return ''.join(['_'+i.lower() if i.isupper()
        else i for i in s]).lstrip('_')

# Create Messages
# Combined system message
try:
    os.mkdir(msg_dir)
except OSError as err:
    print("MSG dir Exists")
    # Remove all current messages
    for m in os.listdir(msg_dir):
        os.remove(os.path.join(msg_dir + "/" + m))

system_msg_path = os.path.join(msg_dir, 'System.msg')
with open(system_msg_path, 'w') as s, open(lim, 'w') as l:
    s.write("# System message - auto generated from kcd\n\n")
    l.write("// Input limits - auto generated from kcd\n\n")
    # Iterate over each message in the KCD file
    for msg in tree.NetworkDefinition.Bus.Message:
        # Create a new ROS message file
        msg_name = msg['name']
        msg_path = os.path.join(msg_dir, '' + msg_name + '.msg')

        names = []
        types = []
        with open(msg_path, 'w') as m:
            m.write("# " + msg_name +
                    " ros message - auto generated from kcd\n\n")

            # Write the message definition to the file
            for sig in msg.Signal:
                # Write the signal definition to the file
                name = camel_to_snake(sig['name'])
                names.append(name)
                size = int(sig['length'])
                if size == 1:
                    var = "bool"
                elif size > 1 and size <= 8:
                    var = "uint8"
                elif size > 8 and size <= 16:
                    var = "uint16"
                elif size > 16 and size <= 32:
                    var = "unt32"
                else:
                    var = "uint64"
                types.append(var)
                if hasattr(sig, 'Value'):
                    if sig.Value['slope'] is not None:
                        var = "float32"
                    if sig.Value['min'] is not None:
                        l.write("double " + camel_to_snake(msg_name).upper()
                                + "_" + name.upper() + "_MIN =  "
                                + sig.Value['min'] + ";\n")
                        if sig.Value['max'] is not None:
                            l.write("double " + camel_to_snake(msg_name).upper()
                                    + "_" + name.upper() + "_MAX = "
                                    + sig.Value['max'] + ";\n")
                m.write(var +  ' ' + name + '\n')

        msgs.append([msg_name, names, types])

        # Add new message to combined message
        s.write(msg_name + " " + camel_to_snake(msg_name) + '\n')

with open(can_c, 'w') as c, open(can_h, 'w') as h, open(ards_h, 'w') as ah:
    # Add stuff to c and h that always needs to be there
    h.write(template.format(can_h))
    h.write("#include <iostream>\n")
    h.write("#include \"david.h\"\n\n")
    c.write(template.format(can_c))
    c.write("#include \"" + can_h + "\"\n")
    c.write("#include <ros/ros.h>\n\n")

    ah.write(template.format(ards_h))
    ah.write("#include \"david.h\"\n")
    ah.write(packet)

    for msg in msgs:
        h.write("#include <motor_bridge/" + msg[0] + ".h>\n")

    h.write('\n')
    for msg in msgs:
        # Add functions to can_h and can_c
        func = ("int pack_msg(const motor_bridge::"
                + msg[0] + "& msg, uint8_t* buff)")
        h.write(func + ";\n")
        c.write(func + " {\n")
        c.write("    david_" + camel_to_snake(msg[0]) + "_t t = {\n")
        for name in msg[1]:
            line = "        ." + name + " = "
            line += "david_" + camel_to_snake(msg[0])
            line += "_" + name + "_encode(msg." + name + "),\n"
            c.write(line)

        c.write("    };\n\n")
        c.write("    david_" + camel_to_snake(msg[0]) +
                "_pack(buff, &t, sizeof(t));\n")
        c.write("    return DAVID_" +
                camel_to_snake(msg[0]).upper() + "_FRAME_ID;\n")
        c.write("}\n\n")

        # Unpack functions
        func = ("template <typename T>\nvoid " + camel_to_snake(msg[0])
                + "_dispatch(const CANPacket &packet, T function) {\n")
        ah.write(func)
        ah.write("    if (packet.id == DAVID_" + msg[0] + "_FRAME_ID) {\n")
        ah.write("        struct david_" + camel_to_snake(msg[0]) + "_t t;\n")
        ah.write("        david_" + camel_to_snake(msg[0]) + "_unpack(&t, packet.buff, packet.len);\n")
        ah.write("        function(t);\n    }\n}\n\n")

    # Print helpers
    h.write('\n')
    for msg in msgs:
        func = "std::string printable(const motor_bridge::" + msg[0] + "& msg)"
        h.write(func + ";\n")
        c.write(func + " {\n")
        c.write("    std::stringstream s;");
        c.write("    s << \"" + msg[0]  + " - \";\n")
        for name in msg[1]:
            c.write("    s << \"" + name + ": \" << (int)msg." + name + " << \", \";\n")
        c.write("    s << std::endl;\n")
        c.write("    return s.str();\n")
        c.write("}\n\n")

# Add messsage files to cmake
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
new_msgs.append('    System.msg\n')
for msg in msgs:
    new_msgs.append('    ' + msg[0] + '.msg\n')

data[i1:i2] = new_msgs

with open(cmake, 'w') as f:
    for line in data:
        f.write(line)
