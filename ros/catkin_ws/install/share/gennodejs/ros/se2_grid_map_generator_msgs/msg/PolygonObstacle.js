// Auto-generated. Do not edit!

// (in-package se2_grid_map_generator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Polygon2D = require('./Polygon2D.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PolygonObstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.polygon = null;
      this.layers = null;
      this.values = null;
    }
    else {
      if (initObj.hasOwnProperty('polygon')) {
        this.polygon = initObj.polygon
      }
      else {
        this.polygon = new Polygon2D();
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
    // Serializes a message object of type PolygonObstacle
    // Serialize message field [polygon]
    bufferOffset = Polygon2D.serialize(obj.polygon, buffer, bufferOffset);
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
    //deserializes a message object of type PolygonObstacle
    let len;
    let data = new PolygonObstacle(null);
    // Deserialize message field [polygon]
    data.polygon = Polygon2D.deserialize(buffer, bufferOffset);
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
    length += Polygon2D.getMessageSize(object.polygon);
    object.layers.forEach((val) => {
      length += std_msgs.msg.String.getMessageSize(val);
    });
    length += 8 * object.values.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'se2_grid_map_generator_msgs/PolygonObstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c82e92cc64d3dead3fb26d6715443026';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new PolygonObstacle(null);
    if (msg.polygon !== undefined) {
      resolved.polygon = Polygon2D.Resolve(msg.polygon)
    }
    else {
      resolved.polygon = new Polygon2D()
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

module.exports = PolygonObstacle;
