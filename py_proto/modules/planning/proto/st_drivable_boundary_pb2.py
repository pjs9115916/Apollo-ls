# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/st_drivable_boundary.proto

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
  name='modules/planning/proto/st_drivable_boundary.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n1modules/planning/proto/st_drivable_boundary.proto\x12\x0f\x61pollo.planning\"s\n\x1aSTDrivableBoundaryInstance\x12\t\n\x01t\x18\x01 \x01(\x01\x12\x0f\n\x07s_lower\x18\x02 \x01(\x01\x12\x0f\n\x07s_upper\x18\x03 \x01(\x01\x12\x13\n\x0bv_obs_lower\x18\x04 \x01(\x01\x12\x13\n\x0bv_obs_upper\x18\x05 \x01(\x01\"V\n\x12STDrivableBoundary\x12@\n\x0bst_boundary\x18\x01 \x03(\x0b\x32+.apollo.planning.STDrivableBoundaryInstance')
)




_STDRIVABLEBOUNDARYINSTANCE = _descriptor.Descriptor(
  name='STDrivableBoundaryInstance',
  full_name='apollo.planning.STDrivableBoundaryInstance',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='t', full_name='apollo.planning.STDrivableBoundaryInstance.t', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='s_lower', full_name='apollo.planning.STDrivableBoundaryInstance.s_lower', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='s_upper', full_name='apollo.planning.STDrivableBoundaryInstance.s_upper', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='v_obs_lower', full_name='apollo.planning.STDrivableBoundaryInstance.v_obs_lower', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='v_obs_upper', full_name='apollo.planning.STDrivableBoundaryInstance.v_obs_upper', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=70,
  serialized_end=185,
)


_STDRIVABLEBOUNDARY = _descriptor.Descriptor(
  name='STDrivableBoundary',
  full_name='apollo.planning.STDrivableBoundary',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='st_boundary', full_name='apollo.planning.STDrivableBoundary.st_boundary', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=187,
  serialized_end=273,
)

_STDRIVABLEBOUNDARY.fields_by_name['st_boundary'].message_type = _STDRIVABLEBOUNDARYINSTANCE
DESCRIPTOR.message_types_by_name['STDrivableBoundaryInstance'] = _STDRIVABLEBOUNDARYINSTANCE
DESCRIPTOR.message_types_by_name['STDrivableBoundary'] = _STDRIVABLEBOUNDARY
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

STDrivableBoundaryInstance = _reflection.GeneratedProtocolMessageType('STDrivableBoundaryInstance', (_message.Message,), dict(
  DESCRIPTOR = _STDRIVABLEBOUNDARYINSTANCE,
  __module__ = 'modules.planning.proto.st_drivable_boundary_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.STDrivableBoundaryInstance)
  ))
_sym_db.RegisterMessage(STDrivableBoundaryInstance)

STDrivableBoundary = _reflection.GeneratedProtocolMessageType('STDrivableBoundary', (_message.Message,), dict(
  DESCRIPTOR = _STDRIVABLEBOUNDARY,
  __module__ = 'modules.planning.proto.st_drivable_boundary_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.STDrivableBoundary)
  ))
_sym_db.RegisterMessage(STDrivableBoundary)


# @@protoc_insertion_point(module_scope)
