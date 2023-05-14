import can
import cantools
import sys

if (len(sys.argv) != 2):
    print('Error must specify the path to the kcd file')
    exit(1)


db = cantools.database.load_file(sys.argv[1])
bus_name = input("Enter bus name: ")

if (bus_name not in ["can0", 'vcan0']):
    print("Error must be can0 or vcan0")
    exit(1)

can_bus = can.interface.Bus(bus_name, bustype='socketcan')
print("To use call the send(name, data) function with a message name and dictionary with the data")

print("Example:")
print(r"send('PitchCtrl', {'SetPoint': 0.0, 'LeftOffset':0.0, 'RightOffset':0.0})'")

def send(name: str, data: dict):
    msg = db.get_message_by_name(name)
    can_bus.send(can.Message(arbitration_id=msg.frame_id, data=msg.encode(data)))


