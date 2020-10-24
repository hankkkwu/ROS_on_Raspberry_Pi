// Auto-generated. Do not edit!

// (in-package object_detection.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoundingBox = require('./BoundingBox.js');

//-----------------------------------------------------------

class BoundingBoxes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.BoundingBoxes = null;
    }
    else {
      if (initObj.hasOwnProperty('BoundingBoxes')) {
        this.BoundingBoxes = initObj.BoundingBoxes
      }
      else {
        this.BoundingBoxes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoundingBoxes
    // Serialize message field [BoundingBoxes]
    // Serialize the length for message field [BoundingBoxes]
    bufferOffset = _serializer.uint32(obj.BoundingBoxes.length, buffer, bufferOffset);
    obj.BoundingBoxes.forEach((val) => {
      bufferOffset = BoundingBox.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoundingBoxes
    let len;
    let data = new BoundingBoxes(null);
    // Deserialize message field [BoundingBoxes]
    // Deserialize array length for message field [BoundingBoxes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.BoundingBoxes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.BoundingBoxes[i] = BoundingBox.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.BoundingBoxes.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_detection/BoundingBoxes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a68dab7e3456d8b1363c13112d36861c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BoundingBox[] BoundingBoxes
    
    ================================================================================
    MSG: object_detection/BoundingBox
    float32 center_x
    float32 center_y
    float32 width
    float32 height
    float32 confidence
    float32 class_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoundingBoxes(null);
    if (msg.BoundingBoxes !== undefined) {
      resolved.BoundingBoxes = new Array(msg.BoundingBoxes.length);
      for (let i = 0; i < resolved.BoundingBoxes.length; ++i) {
        resolved.BoundingBoxes[i] = BoundingBox.Resolve(msg.BoundingBoxes[i]);
      }
    }
    else {
      resolved.BoundingBoxes = []
    }

    return resolved;
    }
};

module.exports = BoundingBoxes;
