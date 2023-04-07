// Auto-generated. Do not edit!

// (in-package se2_grid_map_generator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Circle2D = require('./Circle2D.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CircularObstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.circle = null;
      this.layers = null;
      this.values = null;
    }
    else {
      if (initObj.hasOwnProperty('circle')) {
        this.circle = initObj.circle
      }
      else {
        this.circle = new Circle2D();
      }
      if (initObj.hasOwnProperty('layers')) {
        this.layers = initObj.layers
      }
      else {
        this.layers = [];
      }
      if (initObj.hasOwnProperty('values')) {
        this.values = initObj.values
      }
      else {
        this.values = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CircularObstacle
    // Serialize message field [circle]
    bufferOffset = Circle2D.serialize(obj.circle, buffer, bufferOffset);
    // Serialize message field [layers]
    // Serialize the length for message field [layers]
    bufferOffset = _serializer.uint32(obj.layers.length, buffer, bufferOffset);
    obj.layers.forEach((val) => {
      bufferOffset = std_msgs.msg.String.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [values]
    // Serialize the length for message field [values]
    bufferOffset = _serializer.uint32(obj.values.length, buffer, bufferOffset);
    obj.values.forEach((val) => {
      bufferOffset = std_msgs.msg.Float64.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CircularObstacle
    let len;
    let data = new CircularObstacle(null);
    // Deserialize message field [circle]
    data.circle = Circle2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [layers]
    // Deserialize array length for message field [layers]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.layers = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.layers[i] = std_msgs.msg.String.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [values]
    // Deserialize array length for message field [values]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.values = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.values[i] = std_msgs.msg.Float64.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.layers.forEach((val) => {
      length += std_msgs.msg.String.getMessageSize(val);
    });
    length += 8 * object.values.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'se2_grid_map_generator_msgs/CircularObstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd3e815e4c24596dee9474400db6e2dc7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new CircularObstacle(null);
    if (msg.circle !== undefined) {
      resolved.circle = Circle2D.Resolve(msg.circle)
    }
    else {
      resolved.circle = new Circle2D()
    }

    if (msg.layers !== undefined) {
      resolved.layers = new Array(msg.layers.length);
      for (let i = 0; i < resolved.layers.length; ++i) {
        resolved.layers[i] = std_msgs.msg.String.Resolve(msg.layers[i]);
      }
    }
    else {
      resolved.layers = []
    }

    if (msg.values !== undefined) {
      resolved.values = new Array(msg.values.length);
      for (let i = 0; i < resolved.values.length; ++i) {
        resolved.values[i] = std_msgs.msg.Float64.Resolve(msg.values[i]);
      }
    }
    else {
      resolved.values = []
    }

    return resolved;
    }
};

module.exports = CircularObstacle;
