# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/piecewise_jerk_path_config.proto

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
  name='modules/planning/proto/piecewise_jerk_path_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n7modules/planning/proto/piecewise_jerk_path_config.proto\x12\x0f\x61pollo.planning\"\xad\x01\n\x17PiecewiseJerkPathConfig\x12\x46\n\x13\x64\x65\x66\x61ult_path_config\x18\x01 \x01(\x0b\x32).apollo.planning.PiecewiseJerkPathWeights\x12J\n\x17lane_change_path_config\x18\x02 \x01(\x0b\x32).apollo.planning.PiecewiseJerkPathWeights\"}\n\x18PiecewiseJerkPathWeights\x12\x13\n\x08l_weight\x18\x01 \x01(\x01:\x01\x31\x12\x16\n\tdl_weight\x18\x02 \x01(\x01:\x03\x31\x30\x30\x12\x18\n\nddl_weight\x18\x03 \x01(\x01:\x04\x31\x30\x30\x30\x12\x1a\n\x0b\x64\x64\x64l_weight\x18\x04 \x01(\x01:\x05\x31\x30\x30\x30\x30')
)




_PIECEWISEJERKPATHCONFIG = _descriptor.Descriptor(
  name='PiecewiseJerkPathConfig',
  full_name='apollo.planning.PiecewiseJerkPathConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='default_path_config', full_name='apollo.planning.PiecewiseJerkPathConfig.default_path_config', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lane_change_path_config', full_name='apollo.planning.PiecewiseJerkPathConfig.lane_change_path_config', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=77,
  serialized_end=250,
)


_PIECEWISEJERKPATHWEIGHTS = _descriptor.Descriptor(
  name='PiecewiseJerkPathWeights',
  full_name='apollo.planning.PiecewiseJerkPathWeights',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='l_weight', full_name='apollo.planning.PiecewiseJerkPathWeights.l_weight', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='dl_weight', full_name='apollo.planning.PiecewiseJerkPathWeights.dl_weight', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(100),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ddl_weight', full_name='apollo.planning.PiecewiseJerkPathWeights.ddl_weight', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1000),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='dddl_weight', full_name='apollo.planning.PiecewiseJerkPathWeights.dddl_weight', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10000),
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
  serialized_start=252,
  serialized_end=377,
)

_PIECEWISEJERKPATHCONFIG.fields_by_name['default_path_config'].message_type = _PIECEWISEJERKPATHWEIGHTS
_PIECEWISEJERKPATHCONFIG.fields_by_name['lane_change_path_config'].message_type = _PIECEWISEJERKPATHWEIGHTS
DESCRIPTOR.message_types_by_name['PiecewiseJerkPathConfig'] = _PIECEWISEJERKPATHCONFIG
DESCRIPTOR.message_types_by_name['PiecewiseJerkPathWeights'] = _PIECEWISEJERKPATHWEIGHTS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PiecewiseJerkPathConfig = _reflection.GeneratedProtocolMessageType('PiecewiseJerkPathConfig', (_message.Message,), dict(
  DESCRIPTOR = _PIECEWISEJERKPATHCONFIG,
  __module__ = 'modules.planning.proto.piecewise_jerk_path_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.PiecewiseJerkPathConfig)
  ))
_sym_db.RegisterMessage(PiecewiseJerkPathConfig)

PiecewiseJerkPathWeights = _reflection.GeneratedProtocolMessageType('PiecewiseJerkPathWeights', (_message.Message,), dict(
  DESCRIPTOR = _PIECEWISEJERKPATHWEIGHTS,
  __module__ = 'modules.planning.proto.piecewise_jerk_path_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.PiecewiseJerkPathWeights)
  ))
_sym_db.RegisterMessage(PiecewiseJerkPathWeights)


# @@protoc_insertion_point(module_scope)