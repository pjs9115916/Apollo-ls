# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/piecewise_jerk_speed_config.proto

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
  name='modules/planning/proto/piecewise_jerk_speed_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n8modules/planning/proto/piecewise_jerk_speed_config.proto\x12\x0f\x61pollo.planning\"\xa2\x01\n\x18PiecewiseJerkSpeedConfig\x12\x15\n\nacc_weight\x18\x01 \x01(\x01:\x01\x31\x12\x17\n\x0bjerk_weight\x18\x02 \x01(\x01:\x02\x31\x30\x12\"\n\x14kappa_penalty_weight\x18\x03 \x01(\x01:\x04\x31\x30\x30\x30\x12\x18\n\x0cref_s_weight\x18\x04 \x01(\x01:\x02\x31\x30\x12\x18\n\x0cref_v_weight\x18\x05 \x01(\x01:\x02\x31\x30')
)




_PIECEWISEJERKSPEEDCONFIG = _descriptor.Descriptor(
  name='PiecewiseJerkSpeedConfig',
  full_name='apollo.planning.PiecewiseJerkSpeedConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='acc_weight', full_name='apollo.planning.PiecewiseJerkSpeedConfig.acc_weight', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='jerk_weight', full_name='apollo.planning.PiecewiseJerkSpeedConfig.jerk_weight', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='kappa_penalty_weight', full_name='apollo.planning.PiecewiseJerkSpeedConfig.kappa_penalty_weight', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1000),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ref_s_weight', full_name='apollo.planning.PiecewiseJerkSpeedConfig.ref_s_weight', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ref_v_weight', full_name='apollo.planning.PiecewiseJerkSpeedConfig.ref_v_weight', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
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
  serialized_start=78,
  serialized_end=240,
)

DESCRIPTOR.message_types_by_name['PiecewiseJerkSpeedConfig'] = _PIECEWISEJERKSPEEDCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PiecewiseJerkSpeedConfig = _reflection.GeneratedProtocolMessageType('PiecewiseJerkSpeedConfig', (_message.Message,), dict(
  DESCRIPTOR = _PIECEWISEJERKSPEEDCONFIG,
  __module__ = 'modules.planning.proto.piecewise_jerk_speed_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.PiecewiseJerkSpeedConfig)
  ))
_sym_db.RegisterMessage(PiecewiseJerkSpeedConfig)


# @@protoc_insertion_point(module_scope)
