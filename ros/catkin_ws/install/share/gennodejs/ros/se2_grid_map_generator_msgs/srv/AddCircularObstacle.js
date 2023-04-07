// Auto-generated. Do not edit!

// (in-package se2_grid_map_generator_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CircularObstacle = require('../msg/CircularObstacle.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class AddCircularObstacleRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.obstacle = null;
    }
    else {
      if (initObj.hasOwnProperty('obstacle')) {
        this.obstacle = initObj.obstacle
      }
      else {
        this.obstacle = new CircularObstacle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddCircularObstacleRequest
    // Serialize message field [obstacle]
    bufferOffset = CircularObstacle.serialize(obj.obstacle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddCircularObstacleRequest
    let len;
    let data = new AddCircularObstacleRequest(null);
    // Deserialize message field [obstacle]
    data.obstacle = CircularObstacle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CircularObstacle.getMessageSize(object.obstacle);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_grid_map_generator_msgs/AddCircularObstacleRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '439d75d797d3b32eca34874ea1d64d5d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request
    se2_grid_map_generator_msgs/CircularObstacle obstacle
    
    ================================================================================
    MSG: se2_grid_map_generator_msgs/CircularObstacle
    se2_grid_map_generator_msgs/Circle2D circle
    std_msgs/String[] layers
    std_msgs/Float64[] values
    ================================================================================
    MSG: se2_grid_map_generator_msgs/Circle2D
    se2_grid_map_generator_msgs/Position2D center
    std_msgs/Float64 radius
    ================================================================================
    MSG: se2_grid_map_generator_msgs/Position2D
    std_msgs/Float64 x
    std_msgs/Float64 y
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AddCircularObstacleRequest(null);
    if (msg.obstacle !== undefined) {
      resolved.obstacle = CircularObstacle.Resolve(msg.obstacle)
    }
    else {
      resolved.obstacle = new CircularObstacle()
    }

    return resolved;
    }
};

class AddCircularObstacleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddCircularObstacleResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddCircularObstacleResponse
    let len;
    let data = new AddCircularObstacleResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_grid_map_generator_msgs/AddCircularObstacleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AddCircularObstacleResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: AddCircularObstacleRequest,
  Response: AddCircularObstacleResponse,
  md5sum() { return '5c6462e7f4d4eb409c7358cc94ad2577'; },
  datatype() { return 'se2_grid_map_generator_msgs/AddCircularObstacle'; }
};
