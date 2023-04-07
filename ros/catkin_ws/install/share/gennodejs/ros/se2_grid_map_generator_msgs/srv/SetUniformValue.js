// Auto-generated. Do not edit!

// (in-package se2_grid_map_generator_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetUniformValueRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.layers = null;
      this.values = null;
    }
    else {
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
    // Serializes a message object of type SetUniformValueRequest
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
    //deserializes a message object of type SetUniformValueRequest
    let len;
    let data = new SetUniformValueRequest(null);
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
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_grid_map_generator_msgs/SetUniformValueRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4670bceb1fea55bf0df1cc8795202181';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request
    std_msgs/String[] layers
    std_msgs/Float64[] values
    
    ================================================================================
    MSG: std_msgs/String
    string data
    
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
    const resolved = new SetUniformValueRequest(null);
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

class SetUniformValueResponse {
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
    // Serializes a message object of type SetUniformValueResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetUniformValueResponse
    let len;
    let data = new SetUniformValueResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_grid_map_generator_msgs/SetUniformValueResponse';
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
    const resolved = new SetUniformValueResponse(null);
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
  Request: SetUniformValueRequest,
  Response: SetUniformValueResponse,
  md5sum() { return 'e0f18e88964ccbda0238df7be68a4fd8'; },
  datatype() { return 'se2_grid_map_generator_msgs/SetUniformValue'; }
};
