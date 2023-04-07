// Auto-generated. Do not edit!

// (in-package se2_navigation_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PathRequestMsg = require('../msg/PathRequestMsg.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class RequestPathSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pathRequest = null;
    }
    else {
      if (initObj.hasOwnProperty('pathRequest')) {
        this.pathRequest = initObj.pathRequest
      }
      else {
        this.pathRequest = new PathRequestMsg();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RequestPathSrvRequest
    // Serialize message field [pathRequest]
    bufferOffset = PathRequestMsg.serialize(obj.pathRequest, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RequestPathSrvRequest
    let len;
    let data = new RequestPathSrvRequest(null);
    // Deserialize message field [pathRequest]
    data.pathRequest = PathRequestMsg.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 112;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_navigation_msgs/RequestPathSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'db2b254db8276cf2c8fcea29d6fee192';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request
    PathRequestMsg 		pathRequest                          # Query, maybe useful for the future
    
    ================================================================================
    MSG: se2_navigation_msgs/PathRequestMsg
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
    const resolved = new RequestPathSrvRequest(null);
    if (msg.pathRequest !== undefined) {
      resolved.pathRequest = PathRequestMsg.Resolve(msg.pathRequest)
    }
    else {
      resolved.pathRequest = new PathRequestMsg()
    }

    return resolved;
    }
};

class RequestPathSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RequestPathSrvResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RequestPathSrvResponse
    let len;
    let data = new RequestPathSrvResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_navigation_msgs/RequestPathSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a1255d4d998bd4d6585c64639b5ee9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response
    bool status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RequestPathSrvResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = false
    }

    return resolved;
    }
};

module.exports = {
  Request: RequestPathSrvRequest,
  Response: RequestPathSrvResponse,
  md5sum() { return '5539d8cfd1e0b02f407ce0465be5beeb'; },
  datatype() { return 'se2_navigation_msgs/RequestPathSrv'; }
};
