# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/navi_obstacle_decider_config.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/planning/proto/navi_obstacle_decider_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n9modules/planning/proto/navi_obstacle_decider_config.proto\x12\x0f\x61pollo.planning\"\x88\x03\n\x19NaviObstacleDeciderConfig\x12\x1f\n\x12min_nudge_distance\x18\x01 \x01(\x01:\x03\x30.2\x12\x1f\n\x12max_nudge_distance\x18\x02 \x01(\x01:\x03\x31.2\x12%\n\x15max_allow_nudge_speed\x18\x03 \x01(\x01:\x06\x31\x36.667\x12\x1a\n\rsafe_distance\x18\x04 \x01(\x01:\x03\x30.2\x12#\n\x15nudge_allow_tolerance\x18\x05 \x01(\x01:\x04\x30.05\x12\x18\n\rcycles_number\x18\x06 \x01(\r:\x01\x33\x12\x1a\n\x0fjudge_dis_coeff\x18\x07 \x01(\x01:\x01\x32\x12\x1b\n\x0f\x62\x61sis_dis_value\x18\x08 \x01(\x01:\x02\x33\x30\x12#\n\x16lateral_velocity_value\x18\t \x01(\x01:\x03\x30.5\x12%\n\x1aspeed_decider_detect_range\x18\n \x01(\x01:\x01\x31\x12\"\n\x15max_keep_nudge_cycles\x18\x0b \x01(\r:\x03\x31\x30\x30')
)




_NAVIOBSTACLEDECIDERCONFIG = _descriptor.Descriptor(
  name='NaviObstacleDeciderConfig',
  full_name='apollo.planning.NaviObstacleDeciderConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='min_nudge_distance', full_name='apollo.planning.NaviObstacleDeciderConfig.min_nudge_distance', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.2),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_nudge_distance', full_name='apollo.planning.NaviObstacleDeciderConfig.max_nudge_distance', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1.2),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_allow_nudge_speed', full_name='apollo.planning.NaviObstacleDeciderConfig.max_allow_nudge_speed', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(16.667),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='safe_distance', full_name='apollo.planning.NaviObstacleDeciderConfig.safe_distance', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.2),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='nudge_allow_tolerance', full_name='apollo.planning.NaviObstacleDeciderConfig.nudge_allow_tolerance', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.05),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cycles_number', full_name='apollo.planning.NaviObstacleDeciderConfig.cycles_number', index=5,
      number=6, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=3,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='judge_dis_coeff', full_name='apollo.planning.NaviObstacleDeciderConfig.judge_dis_coeff', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(2),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='basis_dis_value', full_name='apollo.planning.NaviObstacleDeciderConfig.basis_dis_value', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(30),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lateral_velocity_value', full_name='apollo.planning.NaviObstacleDeciderConfig.lateral_velocity_value', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.5),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='speed_decider_detect_range', full_name='apollo.planning.NaviObstacleDeciderConfig.speed_decider_detect_range', index=9,
      number=10, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_keep_nudge_cycles', full_name='apollo.planning.NaviObstacleDeciderConfig.max_keep_nudge_cycles', index=10,
      number=11, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=100,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=79,
  serialized_end=471,
)

DESCRIPTOR.message_types_by_name['NaviObstacleDeciderConfig'] = _NAVIOBSTACLEDECIDERCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

NaviObstacleDeciderConfig = _reflection.GeneratedProtocolMessageType('NaviObstacleDeciderConfig', (_message.Message,), dict(
  DESCRIPTOR = _NAVIOBSTACLEDECIDERCONFIG,
  __module__ = 'modules.planning.proto.navi_obstacle_decider_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.NaviObstacleDeciderConfig)
  ))
_sym_db.RegisterMessage(NaviObstacleDeciderConfig)


# @@protoc_insertion_point(module_scope)