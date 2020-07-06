# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/proto/fused_classifier_config.proto

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
  name='modules/perception/proto/fused_classifier_config.proto',
  package='apollo.perception.lidar',
  syntax='proto2',
  serialized_pb=_b('\n6modules/perception/proto/fused_classifier_config.proto\x12\x17\x61pollo.perception.lidar\"\xec\x01\n\x15\x46usedClassifierConfig\x12\x1b\n\x0ftemporal_window\x18\x01 \x01(\x01:\x02\x32\x30\x12$\n\x16\x65nable_temporal_fusion\x18\x02 \x01(\x08:\x04true\x12\x35\n\x16one_shot_fusion_method\x18\x03 \x01(\t:\x15\x43\x43RFOneShotTypeFusion\x12\x36\n\x16sequence_fusion_method\x18\x04 \x01(\t:\x16\x43\x43RFSequenceTypeFusion\x12!\n\x13use_tracked_objects\x18\x05 \x01(\x08:\x04true')
)




_FUSEDCLASSIFIERCONFIG = _descriptor.Descriptor(
  name='FusedClassifierConfig',
  full_name='apollo.perception.lidar.FusedClassifierConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='temporal_window', full_name='apollo.perception.lidar.FusedClassifierConfig.temporal_window', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(20),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='enable_temporal_fusion', full_name='apollo.perception.lidar.FusedClassifierConfig.enable_temporal_fusion', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='one_shot_fusion_method', full_name='apollo.perception.lidar.FusedClassifierConfig.one_shot_fusion_method', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("CCRFOneShotTypeFusion").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sequence_fusion_method', full_name='apollo.perception.lidar.FusedClassifierConfig.sequence_fusion_method', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=True, default_value=_b("CCRFSequenceTypeFusion").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_tracked_objects', full_name='apollo.perception.lidar.FusedClassifierConfig.use_tracked_objects', index=4,
      number=5, type=8, cpp_type=7, label=1,
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
  serialized_start=84,
  serialized_end=320,
)

DESCRIPTOR.message_types_by_name['FusedClassifierConfig'] = _FUSEDCLASSIFIERCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FusedClassifierConfig = _reflection.GeneratedProtocolMessageType('FusedClassifierConfig', (_message.Message,), dict(
  DESCRIPTOR = _FUSEDCLASSIFIERCONFIG,
  __module__ = 'modules.perception.proto.fused_classifier_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.lidar.FusedClassifierConfig)
  ))
_sym_db.RegisterMessage(FusedClassifierConfig)


# @@protoc_insertion_point(module_scope)
