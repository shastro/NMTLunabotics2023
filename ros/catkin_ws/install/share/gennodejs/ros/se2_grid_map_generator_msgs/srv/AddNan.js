// Auto-generated. Do not edit!

// (in-package se2_grid_map_generator_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Polygon2D = require('../msg/Polygon2D.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class AddNanRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.polygon = null;
    }
    else {
      if (initObj.hasOwnProperty('polygon')) {
        this.polygon = initObj.polygon
      }
      else {
        this.polygon = new Polygon2D();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddNanRequest
    // Serialize message field [polygon]
    bufferOffset = Polygon2D.serialize(obj.polygon, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddNanRequest
    let len;
    let data = new AddNanRequest(null);
    // Deserialize message field [polygon]
    data.polygon = Polygon2D.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Polygon2D.getMessageSize(object.polygon);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_grid_map_generator_msgs/AddNanRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4056b8453382bcc22d3fae7b7a4ff29a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request
    se2_grid_map_generator_msgs/Polygon2D polygon
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AddNanRequest(null);
    if (msg.polygon !== undefined) {
      resolved.polygon = Polygon2D.Resolve(msg.polygon)
    }
    else {
      resolved.polygon = new Polygon2D()
    }

    return resolved;
    }
};

class AddNanResponse {
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
    // Serializes a message object of type AddNanResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddNanResponse
    let len;
    let data = new AddNanResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_grid_map_generator_msgs/AddNanResponse';
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
    const resolved = new AddNanResponse(null);
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
  Request: AddNanRequest,
  Response: AddNanResponse,
  md5sum() { return 'ed846c130da1159d54d0e3b43ecc2916'; },
  datatype() { return 'se2_grid_map_generator_msgs/AddNan'; }
};
