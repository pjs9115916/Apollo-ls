# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/open_space_fallback_decider_config.proto

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
  name='modules/planning/proto/open_space_fallback_decider_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n?modules/planning/proto/open_space_fallback_decider_config.proto\x12\x0f\x61pollo.planning\"\xe6\x01\n\x1eOpenSpaceFallBackDeciderConfig\x12,\n!open_space_prediction_time_period\x18\x01 \x01(\x01:\x01\x35\x12\x31\n&open_space_fallback_collision_distance\x18\x02 \x01(\x01:\x01\x35\x12,\n!open_space_fallback_stop_distance\x18\x03 \x01(\x01:\x01\x32\x12\x35\n)open_space_fallback_collision_time_buffer\x18\x04 \x01(\x01:\x02\x31\x30')
)




_OPENSPACEFALLBACKDECIDERCONFIG = _descriptor.Descriptor(
  name='OpenSpaceFallBackDeciderConfig',
  full_name='apollo.planning.OpenSpaceFallBackDeciderConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='open_space_prediction_time_period', full_name='apollo.planning.OpenSpaceFallBackDeciderConfig.open_space_prediction_time_period', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(5),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='open_space_fallback_collision_distance', full_name='apollo.planning.OpenSpaceFallBackDeciderConfig.open_space_fallback_collision_distance', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(5),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='open_space_fallback_stop_distance', full_name='apollo.planning.OpenSpaceFallBackDeciderConfig.open_space_fallback_stop_distance', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(2),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='open_space_fallback_collision_time_buffer', full_name='apollo.planning.OpenSpaceFallBackDeciderConfig.open_space_fallback_collision_time_buffer', index=3,
      number=4, type=1, cpp_type=5, label=1,
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
  serialized_start=85,
  serialized_end=315,
)

DESCRIPTOR.message_types_by_name['OpenSpaceFallBackDeciderConfig'] = _OPENSPACEFALLBACKDECIDERCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

OpenSpaceFallBackDeciderConfig = _reflection.GeneratedProtocolMessageType('OpenSpaceFallBackDeciderConfig', (_message.Message,), dict(
  DESCRIPTOR = _OPENSPACEFALLBACKDECIDERCONFIG,
  __module__ = 'modules.planning.proto.open_space_fallback_decider_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.OpenSpaceFallBackDeciderConfig)
  ))
_sym_db.RegisterMessage(OpenSpaceFallBackDeciderConfig)


# @@protoc_insertion_point(module_scope)
