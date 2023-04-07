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

class PathRequestMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.startingPose = null;
      this.goalPose = null;
    }
    else {
      if (initObj.hasOwnProperty('startingPose')) {
        this.startingPose = initObj.startingPose
      }
      else {
        this.startingPose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('goalPose')) {
        this.goalPose = initObj.goalPose
      }
      else {
        this.goalPose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathRequestMsg
    // Serialize message field [startingPose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.startingPose, buffer, bufferOffset);
    // Serialize message field [goalPose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.goalPose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathRequestMsg
    let len;
    let data = new PathRequestMsg(null);
    // Deserialize message field [startingPose]
    data.startingPose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [goalPose]
    data.goalPose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 112;
  }

  static datatype() {
    // Returns string type for a message object
    return 'se2_navigation_msgs/PathRequestMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e0af408eeb123474bf9e677b140defe0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose startingPose
    geometry_msgs/Pose goalPose
    
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
    const resolved = new PathRequestMsg(null);
    if (msg.startingPose !== undefined) {
      resolved.startingPose = geometry_msgs.msg.Pose.Resolve(msg.startingPose)
    }
    else {
      resolved.startingPose = new geometry_msgs.msg.Pose()
    }

    if (msg.goalPose !== undefined) {
      resolved.goalPose = geometry_msgs.msg.Pose.Resolve(msg.goalPose)
    }
    else {
      resolved.goalPose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = PathRequestMsg;
