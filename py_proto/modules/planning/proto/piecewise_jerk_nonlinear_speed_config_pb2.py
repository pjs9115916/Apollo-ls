# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/piecewise_jerk_nonlinear_speed_config.proto

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
  name='modules/planning/proto/piecewise_jerk_nonlinear_speed_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\nBmodules/planning/proto/piecewise_jerk_nonlinear_speed_config.proto\x12\x0f\x61pollo.planning\"\xd4\x02\n!PiecewiseJerkNonlinearSpeedConfig\x12\x17\n\nacc_weight\x18\x01 \x01(\x01:\x03\x35\x30\x30\x12\x18\n\x0bjerk_weight\x18\x02 \x01(\x01:\x03\x31\x30\x30\x12\x1b\n\x0elat_acc_weight\x18\x03 \x01(\x01:\x03\x35\x30\x30\x12\x1e\n\x12s_potential_weight\x18\x04 \x01(\x01:\x02\x31\x30\x12\x18\n\x0cref_v_weight\x18\x05 \x01(\x01:\x02\x31\x30\x12\x18\n\x0cref_s_weight\x18\x06 \x01(\x01:\x02\x31\x30\x12\x18\n\x0c\x65nd_s_weight\x18\x07 \x01(\x01:\x02\x31\x30\x12\x18\n\x0c\x65nd_v_weight\x18\x08 \x01(\x01:\x02\x31\x30\x12\x18\n\x0c\x65nd_a_weight\x18\t \x01(\x01:\x02\x31\x30\x12\x1f\n\x13soft_s_bound_weight\x18\n \x01(\x01:\x02\x31\x30\x12\x1c\n\x0euse_warm_start\x18\x64 \x01(\x08:\x04true')
)




_PIECEWISEJERKNONLINEARSPEEDCONFIG = _descriptor.Descriptor(
  name='PiecewiseJerkNonlinearSpeedConfig',
  full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='acc_weight', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.acc_weight', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(500),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='jerk_weight', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.jerk_weight', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(100),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lat_acc_weight', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.lat_acc_weight', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(500),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='s_potential_weight', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.s_potential_weight', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ref_v_weight', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.ref_v_weight', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ref_s_weight', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.ref_s_weight', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end_s_weight', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.end_s_weight', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end_v_weight', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.end_v_weight', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end_a_weight', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.end_a_weight', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='soft_s_bound_weight', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.soft_s_bound_weight', index=9,
      number=10, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_warm_start', full_name='apollo.planning.PiecewiseJerkNonlinearSpeedConfig.use_warm_start', index=10,
      number=100, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
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
  serialized_start=88,
  serialized_end=428,
)

DESCRIPTOR.message_types_by_name['PiecewiseJerkNonlinearSpeedConfig'] = _PIECEWISEJERKNONLINEARSPEEDCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PiecewiseJerkNonlinearSpeedConfig = _reflection.GeneratedProtocolMessageType('PiecewiseJerkNonlinearSpeedConfig', (_message.Message,), dict(
  DESCRIPTOR = _PIECEWISEJERKNONLINEARSPEEDCONFIG,
  __module__ = 'modules.planning.proto.piecewise_jerk_nonlinear_speed_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.PiecewiseJerkNonlinearSpeedConfig)
  ))
_sym_db.RegisterMessage(PiecewiseJerkNonlinearSpeedConfig)


# @@protoc_insertion_point(module_scope)
