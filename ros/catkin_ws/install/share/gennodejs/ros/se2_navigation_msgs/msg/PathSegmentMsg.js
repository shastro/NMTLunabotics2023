// Auto-generated. Do not edit!

// (in-package se2_navigation_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class PathSegmentMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drivingDirection = null;
      this.points = null;
    }
    else {
      if (initObj.hasOwnProperty('drivingDirection')) {
        this.drivingDirection = initObj.drivingDirection
      }
      else {
        this.drivingDirection = 0;
      }
      if (initObj.hasOwnProperty('points')) {
        this.points = initObj.points
      }
      else {
        this.points = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathSegmentMsg
    // Serialize message field [drivingDirection]
    bufferOffset = _serializer.int8(obj.drivingDirection, buffer, bufferOffset);
    // Serialize message field [points]
    // Serialize the length for message field [points]
    bufferOffset = _serializer.uint32(obj.points.length, buffer, bufferOffset);
    obj.points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathSegmentMsg
    let len;
    let data = new PathSegmentMsg(null);
    // Deserialize message field [drivingDirection]
    data.drivingDirection = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [points]
    // Deserialize array length for message field [points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.points[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 56 * object.points.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'se2_navigation_msgs/PathSegmentMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8da26ed105802ea1199162a6750daa5e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
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
    const resolved = new PathSegmentMsg(null);
    if (msg.drivingDirection !== undefined) {
      resolved.drivingDirection = msg.drivingDirection;
    }
    else {
      resolved.drivingDirection = 0
    }

    if (msg.points !== undefined) {
      resolved.points = new Array(msg.points.length);
      for (let i = 0; i < resolved.points.length; ++i) {
        resolved.points[i] = geometry_msgs.msg.Pose.Resolve(msg.points[i]);
      }
    }
    else {
      resolved.points = []
    }

    return resolved;
    }
};

// Constants for message
PathSegmentMsg.Constants = {
  FORWARD: 0,
  BACKWARDS: 1,
}

module.exports = PathSegmentMsg;
