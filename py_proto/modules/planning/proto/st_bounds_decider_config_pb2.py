# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/st_bounds_decider_config.proto

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
  name='modules/planning/proto/st_bounds_decider_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n5modules/planning/proto/st_bounds_decider_config.proto\x12\x0f\x61pollo.planning\".\n\x15STBoundsDeciderConfig\x12\x15\n\ntotal_time\x18\x01 \x01(\x01:\x01\x37')
)




_STBOUNDSDECIDERCONFIG = _descriptor.Descriptor(
  name='STBoundsDeciderConfig',
  full_name='apollo.planning.STBoundsDeciderConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='total_time', full_name='apollo.planning.STBoundsDeciderConfig.total_time', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(7),
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
  serialized_start=74,
  serialized_end=120,
)

DESCRIPTOR.message_types_by_name['STBoundsDeciderConfig'] = _STBOUNDSDECIDERCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

STBoundsDeciderConfig = _reflection.GeneratedProtocolMessageType('STBoundsDeciderConfig', (_message.Message,), dict(
  DESCRIPTOR = _STBOUNDSDECIDERCONFIG,
  __module__ = 'modules.planning.proto.st_bounds_decider_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.STBoundsDeciderConfig)
  ))
_sym_db.RegisterMessage(STBoundsDeciderConfig)


# @@protoc_insertion_point(module_scope)
