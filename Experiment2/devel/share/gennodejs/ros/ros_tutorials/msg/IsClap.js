// Auto-generated. Do not edit!

// (in-package ros_tutorials.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class IsClap {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
      this.isclap = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = 0;
      }
      if (initObj.hasOwnProperty('isclap')) {
        this.isclap = initObj.isclap
      }
      else {
        this.isclap = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IsClap
    // Serialize message field [data]
    bufferOffset = _serializer.int32(obj.data, buffer, bufferOffset);
    // Serialize message field [isclap]
    bufferOffset = _serializer.string(obj.isclap, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IsClap
    let len;
    let data = new IsClap(null);
    // Deserialize message field [data]
    data.data = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [isclap]
    data.isclap = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.isclap);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_tutorials/IsClap';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '02936c85ca2c23baf7e87b1b66fbc9e8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 data
    string isclap
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IsClap(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = 0
    }

    if (msg.isclap !== undefined) {
      resolved.isclap = msg.isclap;
    }
    else {
      resolved.isclap = ''
    }

    return resolved;
    }
};

module.exports = IsClap;
