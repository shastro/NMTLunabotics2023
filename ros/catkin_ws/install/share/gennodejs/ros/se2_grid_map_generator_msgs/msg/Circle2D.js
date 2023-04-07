// Auto-generated. Do not edit!

// (in-package se2_grid_map_generator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Position2D = require('./Position2D.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Circle2D {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.center = null;
      this.radius = null;
    }
    else {
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new Position2D();
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Circle2D
    // Serialize message field [center]
    bufferOffset = Position2D.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.radius, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Circle2D
    let len;
    let data = new Circle2D(null);
    // Deserialize message field [center]
    data.center = Position2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'se2_grid_map_generator_msgs/Circle2D';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0dfece0138cac79dd8ef38f9d1fa9947';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    se2_grid_map_generator_msgs/Position2D center
    std_msgs/Float64 radius
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
    const resolved = new Circle2D(null);
    if (msg.center !== undefined) {
      resolved.center = Position2D.Resolve(msg.center)
    }
    else {
      resolved.center = new Position2D()
    }

    if (msg.radius !== undefined) {
      resolved.radius = std_msgs.msg.Float64.Resolve(msg.radius)
    }
    else {
      resolved.radius = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = Circle2D;
