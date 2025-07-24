// Auto-generated. Do not edit!

// (in-package task_mgr.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class task_mgr {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fsm_state = null;
      this.task_list = null;
      this.cur_task_id = null;
      this.cur_task_name = null;
      this.cur_stage = null;
      this.cur_goal = null;
      this.target_object = null;
    }
    else {
      if (initObj.hasOwnProperty('fsm_state')) {
        this.fsm_state = initObj.fsm_state
      }
      else {
        this.fsm_state = '';
      }
      if (initObj.hasOwnProperty('task_list')) {
        this.task_list = initObj.task_list
      }
      else {
        this.task_list = [];
      }
      if (initObj.hasOwnProperty('cur_task_id')) {
        this.cur_task_id = initObj.cur_task_id
      }
      else {
        this.cur_task_id = '';
      }
      if (initObj.hasOwnProperty('cur_task_name')) {
        this.cur_task_name = initObj.cur_task_name
      }
      else {
        this.cur_task_name = '';
      }
      if (initObj.hasOwnProperty('cur_stage')) {
        this.cur_stage = initObj.cur_stage
      }
      else {
        this.cur_stage = '';
      }
      if (initObj.hasOwnProperty('cur_goal')) {
        this.cur_goal = initObj.cur_goal
      }
      else {
        this.cur_goal = '';
      }
      if (initObj.hasOwnProperty('target_object')) {
        this.target_object = initObj.target_object
      }
      else {
        this.target_object = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type task_mgr
    // Serialize message field [fsm_state]
    bufferOffset = _serializer.string(obj.fsm_state, buffer, bufferOffset);
    // Serialize message field [task_list]
    bufferOffset = _arraySerializer.string(obj.task_list, buffer, bufferOffset, null);
    // Serialize message field [cur_task_id]
    bufferOffset = _serializer.string(obj.cur_task_id, buffer, bufferOffset);
    // Serialize message field [cur_task_name]
    bufferOffset = _serializer.string(obj.cur_task_name, buffer, bufferOffset);
    // Serialize message field [cur_stage]
    bufferOffset = _serializer.string(obj.cur_stage, buffer, bufferOffset);
    // Serialize message field [cur_goal]
    bufferOffset = _serializer.string(obj.cur_goal, buffer, bufferOffset);
    // Serialize message field [target_object]
    bufferOffset = _serializer.string(obj.target_object, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type task_mgr
    let len;
    let data = new task_mgr(null);
    // Deserialize message field [fsm_state]
    data.fsm_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [task_list]
    data.task_list = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [cur_task_id]
    data.cur_task_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cur_task_name]
    data.cur_task_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cur_stage]
    data.cur_stage = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cur_goal]
    data.cur_goal = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [target_object]
    data.target_object = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.fsm_state);
    object.task_list.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += _getByteLength(object.cur_task_id);
    length += _getByteLength(object.cur_task_name);
    length += _getByteLength(object.cur_stage);
    length += _getByteLength(object.cur_goal);
    length += _getByteLength(object.target_object);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'task_mgr/task_mgr';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '85e6602b4a8e4a88f3d67a09ef6bcc82';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string fsm_state
    string[] task_list
    string cur_task_id
    string cur_task_name
    string cur_stage
    string cur_goal
    string target_object
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new task_mgr(null);
    if (msg.fsm_state !== undefined) {
      resolved.fsm_state = msg.fsm_state;
    }
    else {
      resolved.fsm_state = ''
    }

    if (msg.task_list !== undefined) {
      resolved.task_list = msg.task_list;
    }
    else {
      resolved.task_list = []
    }

    if (msg.cur_task_id !== undefined) {
      resolved.cur_task_id = msg.cur_task_id;
    }
    else {
      resolved.cur_task_id = ''
    }

    if (msg.cur_task_name !== undefined) {
      resolved.cur_task_name = msg.cur_task_name;
    }
    else {
      resolved.cur_task_name = ''
    }

    if (msg.cur_stage !== undefined) {
      resolved.cur_stage = msg.cur_stage;
    }
    else {
      resolved.cur_stage = ''
    }

    if (msg.cur_goal !== undefined) {
      resolved.cur_goal = msg.cur_goal;
    }
    else {
      resolved.cur_goal = ''
    }

    if (msg.target_object !== undefined) {
      resolved.target_object = msg.target_object;
    }
    else {
      resolved.target_object = ''
    }

    return resolved;
    }
};

module.exports = task_mgr;
