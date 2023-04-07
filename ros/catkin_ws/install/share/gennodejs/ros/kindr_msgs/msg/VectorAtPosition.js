// Auto-generated. Do not edit!

// (in-package kindr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class VectorAtPosition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.type = null;
      this.name = null;
      this.vector = null;
      this.position = null;
      this.position_frame_id = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('vector')) {
        this.vector = initObj.vector
      }
      else {
        this.vector = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('position_frame_id')) {
        this.position_frame_id = initObj.position_frame_id
      }
      else {
        this.position_frame_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VectorAtPosition
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [vector]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.vector, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [position_frame_id]
    bufferOffset = _serializer.string(obj.position_frame_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VectorAtPosition
    let len;
    let data = new VectorAtPosition(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [vector]
    data.vector = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_frame_id]
    data.position_frame_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.name);
    length += _getByteLength(object.position_frame_id);
    return length + 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kindr_msgs/VectorAtPosition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fcf32a1df9f6d53ef1926f20ce6b66e0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Vector type definition (based on Kindr definitions)
    uint8 TYPE_TYPELESS=0
    uint8 TYPE_JERK=6
    uint8 TYPE_ACCELERATION=7
    uint8 TYPE_VELOCITY=8
    uint8 TYPE_POSITION=9
    uint8 TYPE_FORCE=10
    uint8 TYPE_MOMEMTUM=11
    uint8 TYPE_ANGULAR_JERK=12
    uint8 TYPE_ANGULAR_ACCELERATION=13
    uint8 TYPE_ANGULAR_VELOCITY=14
    uint8 TYPE_TORQUE=16
    uint8 TYPE_ANGULAR_MOMEMTUM=17
    
    Header header
    uint8 type
    string name
    geometry_msgs/Vector3 vector # Frame defined in header
    geometry_msgs/Point position # Point of origin of the vector
    string position_frame_id # If empty same as vector frame
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VectorAtPosition(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.vector !== undefined) {
      resolved.vector = geometry_msgs.msg.Vector3.Resolve(msg.vector)
    }
    else {
      resolved.vector = new geometry_msgs.msg.Vector3()
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.position_frame_id !== undefined) {
      resolved.position_frame_id = msg.position_frame_id;
    }
    else {
      resolved.position_frame_id = ''
    }

    return resolved;
    }
};

// Constants for message
VectorAtPosition.Constants = {
  TYPE_TYPELESS: 0,
  TYPE_JERK: 6,
  TYPE_ACCELERATION: 7,
  TYPE_VELOCITY: 8,
  TYPE_POSITION: 9,
  TYPE_FORCE: 10,
  TYPE_MOMEMTUM: 11,
  TYPE_ANGULAR_JERK: 12,
  TYPE_ANGULAR_ACCELERATION: 13,
  TYPE_ANGULAR_VELOCITY: 14,
  TYPE_TORQUE: 16,
  TYPE_ANGULAR_MOMEMTUM: 17,
}

module.exports = VectorAtPosition;
