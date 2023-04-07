// Auto-generated. Do not edit!

// (in-package se2_navigation_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PathSegmentMsg = require('./PathSegmentMsg.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PathMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.segment = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('segment')) {
        this.segment = initObj.segment
      }
      else {
        this.segment = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [segment]
    // Serialize the length for message field [segment]
    bufferOffset = _serializer.uint32(obj.segment.length, buffer, bufferOffset);
    obj.segment.forEach((val) => {
      bufferOffset = PathSegmentMsg.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathMsg
    let len;
    let data = new PathMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [segment]
    // Deserialize array length for message field [segment]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.segment = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.segment[i] = PathSegmentMsg.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.segment.forEach((val) => {
      length += PathSegmentMsg.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'se2_navigation_msgs/PathMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '903b0d0b2bfe4cbcdeb7a06291ea8df4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    std_msgs/Header header
    se2_navigation_msgs/PathSegmentMsg[] segment
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
    MSG: se2_navigation_msgs/PathSegmentMsg
    
    
    int8 FORWARD = 0
    int8 BACKWARDS = 1
    
    int8 drivingDirection
    
    
    geometry_msgs/Pose[] points
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PathMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.segment !== undefined) {
      resolved.segment = new Array(msg.segment.length);
      for (let i = 0; i < resolved.segment.length; ++i) {
        resolved.segment[i] = PathSegmentMsg.Resolve(msg.segment[i]);
      }
    }
    else {
      resolved.segment = []
    }

    return resolved;
    }
};

module.exports = PathMsg;
