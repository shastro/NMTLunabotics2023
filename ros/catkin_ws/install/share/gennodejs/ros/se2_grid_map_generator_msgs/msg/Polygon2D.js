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

//-----------------------------------------------------------

class Polygon2D {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vertices = null;
    }
    else {
      if (initObj.hasOwnProperty('vertices')) {
        this.vertices = initObj.vertices
      }
      else {
        this.vertices = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Polygon2D
    // Serialize message field [vertices]
    // Serialize the length for message field [vertices]
    bufferOffset = _serializer.uint32(obj.vertices.length, buffer, bufferOffset);
    obj.vertices.forEach((val) => {
      bufferOffset = Position2D.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Polygon2D
    let len;
    let data = new Polygon2D(null);
    // Deserialize message field [vertices]
    // Deserialize array length for message field [vertices]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.vertices = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.vertices[i] = Position2D.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.vertices.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'se2_grid_map_generator_msgs/Polygon2D';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bccfee9dbb3170a57e32edc6d3cfcef5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Polygon2D(null);
    if (msg.vertices !== undefined) {
      resolved.vertices = new Array(msg.vertices.length);
      for (let i = 0; i < resolved.vertices.length; ++i) {
        resolved.vertices[i] = Position2D.Resolve(msg.vertices[i]);
      }
    }
    else {
      resolved.vertices = []
    }

    return resolved;
    }
};

module.exports = Polygon2D;
