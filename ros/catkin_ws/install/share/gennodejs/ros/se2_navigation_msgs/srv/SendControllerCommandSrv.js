// Auto-generated. Do not edit!

// (in-package se2_navigation_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControllerCommandMsg = require('../msg/ControllerCommandMsg.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SendControllerCommandSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = new ControllerCommandMsg();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SendControllerCommandSrvRequest
    // Serialize message field [command]
    bufferOffset = ControllerCommandMsg.serialize(obj.command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SendControllerCommandSrvRequest
    let len;
    let data = new SendControllerCommandSrvRequest(null);
    // Deserialize message field [command]
    data.command = ControllerCommandMsg.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_navigation_msgs/SendControllerCommandSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '50569f8fcb12c151e963a874cc25a053';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request
    ControllerCommandMsg command                          # Query, maybe useful for the future
    
    ================================================================================
    MSG: se2_navigation_msgs/ControllerCommandMsg
    
    int8 START_TRACKING=0
    int8 STOP_TRACKING=1
    
    int8 command
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SendControllerCommandSrvRequest(null);
    if (msg.command !== undefined) {
      resolved.command = ControllerCommandMsg.Resolve(msg.command)
    }
    else {
      resolved.command = new ControllerCommandMsg()
    }

    return resolved;
    }
};

class SendControllerCommandSrvResponse {
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
    // Serializes a message object of type SendControllerCommandSrvResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SendControllerCommandSrvResponse
    let len;
    let data = new SendControllerCommandSrvResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'se2_navigation_msgs/SendControllerCommandSrvResponse';
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
    const resolved = new SendControllerCommandSrvResponse(null);
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
  Request: SendControllerCommandSrvRequest,
  Response: SendControllerCommandSrvResponse,
  md5sum() { return '8f41fa61e7f9b864f313c51bd6bcb1ba'; },
  datatype() { return 'se2_navigation_msgs/SendControllerCommandSrv'; }
};
