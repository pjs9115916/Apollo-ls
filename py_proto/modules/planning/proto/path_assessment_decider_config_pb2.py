# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/path_assessment_decider_config.proto

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
  name='modules/planning/proto/path_assessment_decider_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n;modules/planning/proto/path_assessment_decider_config.proto\x12\x0f\x61pollo.planning\"\x1d\n\x1bPathAssessmentDeciderConfig')
)




_PATHASSESSMENTDECIDERCONFIG = _descriptor.Descriptor(
  name='PathAssessmentDeciderConfig',
  full_name='apollo.planning.PathAssessmentDeciderConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
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
  serialized_start=80,
  serialized_end=109,
)

DESCRIPTOR.message_types_by_name['PathAssessmentDeciderConfig'] = _PATHASSESSMENTDECIDERCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PathAssessmentDeciderConfig = _reflection.GeneratedProtocolMessageType('PathAssessmentDeciderConfig', (_message.Message,), dict(
  DESCRIPTOR = _PATHASSESSMENTDECIDERCONFIG,
  __module__ = 'modules.planning.proto.path_assessment_decider_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.PathAssessmentDeciderConfig)
  ))
_sym_db.RegisterMessage(PathAssessmentDeciderConfig)


# @@protoc_insertion_point(module_scope)
