// Auto-generated. Do not edit!

// (in-package se2_grid_map_generator_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PolygonObstacle = require('../msg/PolygonObstacle.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class AddPolygonObstacleRequest {
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
        this.obstacle = new PolygonObstacle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddPolygonObstacleRequest
    // Serialize message field [obstacle]
    bufferOffset = PolygonObstacle.serialize(obj.obstacle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddPolygonObstacleRequest
    let len;
    let data = new AddPolygonObstacleRequest(null);
    // Deserialize message field [obstacle]
    data.obstacle = PolygonObstacle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += PolygonObstacle.getMessageSize(object.obstacle);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_grid_map_generator_msgs/AddPolygonObstacleRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa762edd6cdbe7dacb2bbde47c7da09b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request
    se2_grid_map_generator_msgs/PolygonObstacle obstacle
    
    ================================================================================
    MSG: se2_grid_map_generator_msgs/PolygonObstacle
    se2_grid_map_generator_msgs/Polygon2D polygon
    std_msgs/String[] layers
    std_msgs/Float64[] values
    ================================================================================
    MSG: se2_grid_map_generator_msgs/Polygon2D
    se2_grid_map_generator_msgs/Position2D[] vertices
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
    const resolved = new AddPolygonObstacleRequest(null);
    if (msg.obstacle !== undefined) {
      resolved.obstacle = PolygonObstacle.Resolve(msg.obstacle)
    }
    else {
      resolved.obstacle = new PolygonObstacle()
    }

    return resolved;
    }
};

class AddPolygonObstacleResponse {
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
    // Serializes a message object of type AddPolygonObstacleResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddPolygonObstacleResponse
    let len;
    let data = new AddPolygonObstacleResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_grid_map_generator_msgs/AddPolygonObstacleResponse';
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
    const resolved = new AddPolygonObstacleResponse(null);
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
  Request: AddPolygonObstacleRequest,
  Response: AddPolygonObstacleResponse,
  md5sum() { return '8cdfa34c81ded8fbfe0b3c21eb0b43e9'; },
  datatype() { return 'se2_grid_map_generator_msgs/AddPolygonObstacle'; }
};
