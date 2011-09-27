"""autogenerated by genmsg_py from MotorCommand.msg. Do not edit."""
import roslib.message
import struct

import roslib.rostime

class MotorCommand(roslib.message.Message):
  _md5sum = "184e7774b9da63f4a895f16cb3ea4870"
  _type = "MotorControlMsg/MotorCommand"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

int32 precedence

float32 linear_velocity
float32 angular_velocity


"""
  __slots__ = ['seq','stamp','frame_id','precedence','linear_velocity','angular_velocity']
  _slot_types = ['uint32','time','string','int32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       seq,stamp,frame_id,precedence,linear_velocity,angular_velocity
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(MotorCommand, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.seq is None:
        self.seq = 0
      if self.stamp is None:
        self.stamp = roslib.rostime.Time()
      if self.frame_id is None:
        self.frame_id = ''
      if self.precedence is None:
        self.precedence = 0
      if self.linear_velocity is None:
        self.linear_velocity = 0.
      if self.angular_velocity is None:
        self.angular_velocity = 0.
    else:
      self.seq = 0
      self.stamp = roslib.rostime.Time()
      self.frame_id = ''
      self.precedence = 0
      self.linear_velocity = 0.
      self.angular_velocity = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.seq, _x.stamp.secs, _x.stamp.nsecs))
      _x = self.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      _x = self
      buff.write(_struct_i2f.pack(_x.precedence, _x.linear_velocity, _x.angular_velocity))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.stamp is None:
        self.stamp = roslib.rostime.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.seq, _x.stamp.secs, _x.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.precedence, _x.linear_velocity, _x.angular_velocity,) = _struct_i2f.unpack(str[start:end])
      self.stamp.canon()
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.seq, _x.stamp.secs, _x.stamp.nsecs))
      _x = self.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      _x = self
      buff.write(_struct_i2f.pack(_x.precedence, _x.linear_velocity, _x.angular_velocity))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.stamp is None:
        self.stamp = roslib.rostime.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.seq, _x.stamp.secs, _x.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.precedence, _x.linear_velocity, _x.angular_velocity,) = _struct_i2f.unpack(str[start:end])
      self.stamp.canon()
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3I = struct.Struct("<3I")
_struct_i2f = struct.Struct("<i2f")
