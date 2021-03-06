# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/path_bounds_decider_config.proto

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
  name='modules/planning/proto/path_bounds_decider_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n7modules/planning/proto/path_bounds_decider_config.proto\x12\x0f\x61pollo.planning\"\x96\x02\n\x17PathBoundsDeciderConfig\x12\x19\n\x11is_lane_borrowing\x18\x01 \x01(\x08\x12\x14\n\x0cis_pull_over\x18\x02 \x01(\x08\x12/\n#pull_over_destination_to_adc_buffer\x18\x03 \x01(\x01:\x02\x32\x35\x12\x33\n\'pull_over_destination_to_pathend_buffer\x18\x04 \x01(\x01:\x02\x31\x30\x12(\n\x1apull_over_road_edge_buffer\x18\x05 \x01(\x01:\x04\x30.15\x12:\n-pull_over_approach_lon_distance_adjust_factor\x18\x06 \x01(\x01:\x03\x31.5')
)




_PATHBOUNDSDECIDERCONFIG = _descriptor.Descriptor(
  name='PathBoundsDeciderConfig',
  full_name='apollo.planning.PathBoundsDeciderConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='is_lane_borrowing', full_name='apollo.planning.PathBoundsDeciderConfig.is_lane_borrowing', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='is_pull_over', full_name='apollo.planning.PathBoundsDeciderConfig.is_pull_over', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pull_over_destination_to_adc_buffer', full_name='apollo.planning.PathBoundsDeciderConfig.pull_over_destination_to_adc_buffer', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(25),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pull_over_destination_to_pathend_buffer', full_name='apollo.planning.PathBoundsDeciderConfig.pull_over_destination_to_pathend_buffer', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pull_over_road_edge_buffer', full_name='apollo.planning.PathBoundsDeciderConfig.pull_over_road_edge_buffer', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.15),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pull_over_approach_lon_distance_adjust_factor', full_name='apollo.planning.PathBoundsDeciderConfig.pull_over_approach_lon_distance_adjust_factor', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1.5),
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
  serialized_end=355,
)

DESCRIPTOR.message_types_by_name['PathBoundsDeciderConfig'] = _PATHBOUNDSDECIDERCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PathBoundsDeciderConfig = _reflection.GeneratedProtocolMessageType('PathBoundsDeciderConfig', (_message.Message,), dict(
  DESCRIPTOR = _PATHBOUNDSDECIDERCONFIG,
  __module__ = 'modules.planning.proto.path_bounds_decider_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.PathBoundsDeciderConfig)
  ))
_sym_db.RegisterMessage(PathBoundsDeciderConfig)


# @@protoc_insertion_point(module_scope)
