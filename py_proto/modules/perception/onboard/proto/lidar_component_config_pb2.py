# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/onboard/proto/lidar_component_config.proto

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
  name='modules/perception/onboard/proto/lidar_component_config.proto',
  package='apollo.perception.onboard',
  syntax='proto2',
  serialized_pb=_b('\n=modules/perception/onboard/proto/lidar_component_config.proto\x12\x19\x61pollo.perception.onboard\"\xb3\x01\n LidarSegmentationComponentConfig\x12\x13\n\x0bsensor_name\x18\x01 \x01(\t\x12\x14\n\x0c\x65nable_hdmap\x18\x02 \x01(\x08\x12\x1d\n\x15lidar_query_tf_offset\x18\x03 \x01(\x01\x12(\n lidar2novatel_tf2_child_frame_id\x18\x04 \x01(\t\x12\x1b\n\x13output_channel_name\x18\x05 \x01(\t\"X\n\x1fLidarRecognitionComponentConfig\x12\x18\n\x10main_sensor_name\x18\x01 \x01(\t\x12\x1b\n\x13output_channel_name\x18\x02 \x01(\t')
)




_LIDARSEGMENTATIONCOMPONENTCONFIG = _descriptor.Descriptor(
  name='LidarSegmentationComponentConfig',
  full_name='apollo.perception.onboard.LidarSegmentationComponentConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sensor_name', full_name='apollo.perception.onboard.LidarSegmentationComponentConfig.sensor_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='enable_hdmap', full_name='apollo.perception.onboard.LidarSegmentationComponentConfig.enable_hdmap', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lidar_query_tf_offset', full_name='apollo.perception.onboard.LidarSegmentationComponentConfig.lidar_query_tf_offset', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lidar2novatel_tf2_child_frame_id', full_name='apollo.perception.onboard.LidarSegmentationComponentConfig.lidar2novatel_tf2_child_frame_id', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='output_channel_name', full_name='apollo.perception.onboard.LidarSegmentationComponentConfig.output_channel_name', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
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
  serialized_start=93,
  serialized_end=272,
)


_LIDARRECOGNITIONCOMPONENTCONFIG = _descriptor.Descriptor(
  name='LidarRecognitionComponentConfig',
  full_name='apollo.perception.onboard.LidarRecognitionComponentConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='main_sensor_name', full_name='apollo.perception.onboard.LidarRecognitionComponentConfig.main_sensor_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='output_channel_name', full_name='apollo.perception.onboard.LidarRecognitionComponentConfig.output_channel_name', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
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
  serialized_start=274,
  serialized_end=362,
)

DESCRIPTOR.message_types_by_name['LidarSegmentationComponentConfig'] = _LIDARSEGMENTATIONCOMPONENTCONFIG
DESCRIPTOR.message_types_by_name['LidarRecognitionComponentConfig'] = _LIDARRECOGNITIONCOMPONENTCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LidarSegmentationComponentConfig = _reflection.GeneratedProtocolMessageType('LidarSegmentationComponentConfig', (_message.Message,), dict(
  DESCRIPTOR = _LIDARSEGMENTATIONCOMPONENTCONFIG,
  __module__ = 'modules.perception.onboard.proto.lidar_component_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.onboard.LidarSegmentationComponentConfig)
  ))
_sym_db.RegisterMessage(LidarSegmentationComponentConfig)

LidarRecognitionComponentConfig = _reflection.GeneratedProtocolMessageType('LidarRecognitionComponentConfig', (_message.Message,), dict(
  DESCRIPTOR = _LIDARRECOGNITIONCOMPONENTCONFIG,
  __module__ = 'modules.perception.onboard.proto.lidar_component_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.onboard.LidarRecognitionComponentConfig)
  ))
_sym_db.RegisterMessage(LidarRecognitionComponentConfig)


# @@protoc_insertion_point(module_scope)
