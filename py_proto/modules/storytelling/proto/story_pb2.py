# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/storytelling/proto/story.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common.proto import header_pb2 as modules_dot_common_dot_proto_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/storytelling/proto/story.proto',
  package='apollo.storytelling',
  syntax='proto2',
  serialized_pb=_b('\n&modules/storytelling/proto/story.proto\x12\x13\x61pollo.storytelling\x1a!modules/common/proto/header.proto\"\x14\n\x12ProceedWithCaution\"=\n\x0f\x43loseToJunction\x12\x15\n\x08\x64istance\x18\x01 \x01(\x01:\x03nan\x12\x13\n\x0bjunction_id\x18\x02 \x01(\t\"\xb8\x01\n\x07Stories\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\x45\n\x14proceed_with_caution\x18\x02 \x01(\x0b\x32\'.apollo.storytelling.ProceedWithCaution\x12?\n\x11\x63lose_to_junction\x18\x03 \x01(\x0b\x32$.apollo.storytelling.CloseToJunction')
  ,
  dependencies=[modules_dot_common_dot_proto_dot_header__pb2.DESCRIPTOR,])




_PROCEEDWITHCAUTION = _descriptor.Descriptor(
  name='ProceedWithCaution',
  full_name='apollo.storytelling.ProceedWithCaution',
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
  serialized_start=98,
  serialized_end=118,
)


_CLOSETOJUNCTION = _descriptor.Descriptor(
  name='CloseToJunction',
  full_name='apollo.storytelling.CloseToJunction',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='distance', full_name='apollo.storytelling.CloseToJunction.distance', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=(1e10000 * 0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='junction_id', full_name='apollo.storytelling.CloseToJunction.junction_id', index=1,
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
  serialized_start=120,
  serialized_end=181,
)


_STORIES = _descriptor.Descriptor(
  name='Stories',
  full_name='apollo.storytelling.Stories',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.storytelling.Stories.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='proceed_with_caution', full_name='apollo.storytelling.Stories.proceed_with_caution', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='close_to_junction', full_name='apollo.storytelling.Stories.close_to_junction', index=2,
      number=3, type=11, cpp_type=10, label=1,
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
  serialized_start=184,
  serialized_end=368,
)

_STORIES.fields_by_name['header'].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_STORIES.fields_by_name['proceed_with_caution'].message_type = _PROCEEDWITHCAUTION
_STORIES.fields_by_name['close_to_junction'].message_type = _CLOSETOJUNCTION
DESCRIPTOR.message_types_by_name['ProceedWithCaution'] = _PROCEEDWITHCAUTION
DESCRIPTOR.message_types_by_name['CloseToJunction'] = _CLOSETOJUNCTION
DESCRIPTOR.message_types_by_name['Stories'] = _STORIES
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ProceedWithCaution = _reflection.GeneratedProtocolMessageType('ProceedWithCaution', (_message.Message,), dict(
  DESCRIPTOR = _PROCEEDWITHCAUTION,
  __module__ = 'modules.storytelling.proto.story_pb2'
  # @@protoc_insertion_point(class_scope:apollo.storytelling.ProceedWithCaution)
  ))
_sym_db.RegisterMessage(ProceedWithCaution)

CloseToJunction = _reflection.GeneratedProtocolMessageType('CloseToJunction', (_message.Message,), dict(
  DESCRIPTOR = _CLOSETOJUNCTION,
  __module__ = 'modules.storytelling.proto.story_pb2'
  # @@protoc_insertion_point(class_scope:apollo.storytelling.CloseToJunction)
  ))
_sym_db.RegisterMessage(CloseToJunction)

Stories = _reflection.GeneratedProtocolMessageType('Stories', (_message.Message,), dict(
  DESCRIPTOR = _STORIES,
  __module__ = 'modules.storytelling.proto.story_pb2'
  # @@protoc_insertion_point(class_scope:apollo.storytelling.Stories)
  ))
_sym_db.RegisterMessage(Stories)


# @@protoc_insertion_point(module_scope)
