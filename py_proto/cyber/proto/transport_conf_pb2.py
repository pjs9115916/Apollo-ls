# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: cyber/proto/transport_conf.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='cyber/proto/transport_conf.proto',
  package='apollo.cyber.proto',
  syntax='proto2',
  serialized_pb=_b('\n cyber/proto/transport_conf.proto\x12\x12\x61pollo.cyber.proto\"/\n\x13ShmMulticastLocator\x12\n\n\x02ip\x18\x01 \x01(\t\x12\x0c\n\x04port\x18\x02 \x01(\r\"p\n\x07ShmConf\x12\x15\n\rnotifier_type\x18\x01 \x01(\t\x12\x10\n\x08shm_type\x18\x02 \x01(\t\x12<\n\x0bshm_locator\x18\x03 \x01(\x0b\x32\'.apollo.cyber.proto.ShmMulticastLocator\"\x88\x01\n\x13RtpsParticipantAttr\x12\x1a\n\x0elease_duration\x18\x01 \x01(\x05:\x02\x31\x32\x12\x1e\n\x13\x61nnouncement_period\x18\x02 \x01(\x05:\x01\x33\x12\x1b\n\x0e\x64omain_id_gain\x18\x03 \x01(\r:\x03\x32\x30\x30\x12\x18\n\tport_base\x18\x04 \x01(\r:\x05\x31\x30\x30\x30\x30\"\xc4\x01\n\x11\x43ommunicationMode\x12:\n\tsame_proc\x18\x01 \x01(\x0e\x32 .apollo.cyber.proto.OptionalMode:\x05INTRA\x12\x38\n\tdiff_proc\x18\x02 \x01(\x0e\x32 .apollo.cyber.proto.OptionalMode:\x03SHM\x12\x39\n\tdiff_host\x18\x03 \x01(\x0e\x32 .apollo.cyber.proto.OptionalMode:\x04RTPS\"0\n\rResourceLimit\x12\x1f\n\x11max_history_depth\x18\x01 \x01(\r:\x04\x31\x30\x30\x30\"\xff\x01\n\rTransportConf\x12-\n\x08shm_conf\x18\x01 \x01(\x0b\x32\x1b.apollo.cyber.proto.ShmConf\x12\x41\n\x10participant_attr\x18\x02 \x01(\x0b\x32\'.apollo.cyber.proto.RtpsParticipantAttr\x12\x41\n\x12\x63ommunication_mode\x18\x03 \x01(\x0b\x32%.apollo.cyber.proto.CommunicationMode\x12\x39\n\x0eresource_limit\x18\x04 \x01(\x0b\x32!.apollo.cyber.proto.ResourceLimit*8\n\x0cOptionalMode\x12\n\n\x06HYBRID\x10\x00\x12\t\n\x05INTRA\x10\x01\x12\x07\n\x03SHM\x10\x02\x12\x08\n\x04RTPS\x10\x03')
)

_OPTIONALMODE = _descriptor.EnumDescriptor(
  name='OptionalMode',
  full_name='apollo.cyber.proto.OptionalMode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='HYBRID', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INTRA', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SHM', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RTPS', index=3, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=865,
  serialized_end=921,
)
_sym_db.RegisterEnumDescriptor(_OPTIONALMODE)

OptionalMode = enum_type_wrapper.EnumTypeWrapper(_OPTIONALMODE)
HYBRID = 0
INTRA = 1
SHM = 2
RTPS = 3



_SHMMULTICASTLOCATOR = _descriptor.Descriptor(
  name='ShmMulticastLocator',
  full_name='apollo.cyber.proto.ShmMulticastLocator',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ip', full_name='apollo.cyber.proto.ShmMulticastLocator.ip', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='port', full_name='apollo.cyber.proto.ShmMulticastLocator.port', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=56,
  serialized_end=103,
)


_SHMCONF = _descriptor.Descriptor(
  name='ShmConf',
  full_name='apollo.cyber.proto.ShmConf',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='notifier_type', full_name='apollo.cyber.proto.ShmConf.notifier_type', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='shm_type', full_name='apollo.cyber.proto.ShmConf.shm_type', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='shm_locator', full_name='apollo.cyber.proto.ShmConf.shm_locator', index=2,
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
  serialized_start=105,
  serialized_end=217,
)


_RTPSPARTICIPANTATTR = _descriptor.Descriptor(
  name='RtpsParticipantAttr',
  full_name='apollo.cyber.proto.RtpsParticipantAttr',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lease_duration', full_name='apollo.cyber.proto.RtpsParticipantAttr.lease_duration', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=12,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='announcement_period', full_name='apollo.cyber.proto.RtpsParticipantAttr.announcement_period', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=3,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='domain_id_gain', full_name='apollo.cyber.proto.RtpsParticipantAttr.domain_id_gain', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=200,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='port_base', full_name='apollo.cyber.proto.RtpsParticipantAttr.port_base', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=10000,
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
  serialized_start=220,
  serialized_end=356,
)


_COMMUNICATIONMODE = _descriptor.Descriptor(
  name='CommunicationMode',
  full_name='apollo.cyber.proto.CommunicationMode',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='same_proc', full_name='apollo.cyber.proto.CommunicationMode.same_proc', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='diff_proc', full_name='apollo.cyber.proto.CommunicationMode.diff_proc', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=2,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='diff_host', full_name='apollo.cyber.proto.CommunicationMode.diff_host', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=3,
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
  serialized_start=359,
  serialized_end=555,
)


_RESOURCELIMIT = _descriptor.Descriptor(
  name='ResourceLimit',
  full_name='apollo.cyber.proto.ResourceLimit',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='max_history_depth', full_name='apollo.cyber.proto.ResourceLimit.max_history_depth', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=1000,
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
  serialized_start=557,
  serialized_end=605,
)


_TRANSPORTCONF = _descriptor.Descriptor(
  name='TransportConf',
  full_name='apollo.cyber.proto.TransportConf',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='shm_conf', full_name='apollo.cyber.proto.TransportConf.shm_conf', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='participant_attr', full_name='apollo.cyber.proto.TransportConf.participant_attr', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='communication_mode', full_name='apollo.cyber.proto.TransportConf.communication_mode', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='resource_limit', full_name='apollo.cyber.proto.TransportConf.resource_limit', index=3,
      number=4, type=11, cpp_type=10, label=1,
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
  serialized_start=608,
  serialized_end=863,
)

_SHMCONF.fields_by_name['shm_locator'].message_type = _SHMMULTICASTLOCATOR
_COMMUNICATIONMODE.fields_by_name['same_proc'].enum_type = _OPTIONALMODE
_COMMUNICATIONMODE.fields_by_name['diff_proc'].enum_type = _OPTIONALMODE
_COMMUNICATIONMODE.fields_by_name['diff_host'].enum_type = _OPTIONALMODE
_TRANSPORTCONF.fields_by_name['shm_conf'].message_type = _SHMCONF
_TRANSPORTCONF.fields_by_name['participant_attr'].message_type = _RTPSPARTICIPANTATTR
_TRANSPORTCONF.fields_by_name['communication_mode'].message_type = _COMMUNICATIONMODE
_TRANSPORTCONF.fields_by_name['resource_limit'].message_type = _RESOURCELIMIT
DESCRIPTOR.message_types_by_name['ShmMulticastLocator'] = _SHMMULTICASTLOCATOR
DESCRIPTOR.message_types_by_name['ShmConf'] = _SHMCONF
DESCRIPTOR.message_types_by_name['RtpsParticipantAttr'] = _RTPSPARTICIPANTATTR
DESCRIPTOR.message_types_by_name['CommunicationMode'] = _COMMUNICATIONMODE
DESCRIPTOR.message_types_by_name['ResourceLimit'] = _RESOURCELIMIT
DESCRIPTOR.message_types_by_name['TransportConf'] = _TRANSPORTCONF
DESCRIPTOR.enum_types_by_name['OptionalMode'] = _OPTIONALMODE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ShmMulticastLocator = _reflection.GeneratedProtocolMessageType('ShmMulticastLocator', (_message.Message,), dict(
  DESCRIPTOR = _SHMMULTICASTLOCATOR,
  __module__ = 'cyber.proto.transport_conf_pb2'
  # @@protoc_insertion_point(class_scope:apollo.cyber.proto.ShmMulticastLocator)
  ))
_sym_db.RegisterMessage(ShmMulticastLocator)

ShmConf = _reflection.GeneratedProtocolMessageType('ShmConf', (_message.Message,), dict(
  DESCRIPTOR = _SHMCONF,
  __module__ = 'cyber.proto.transport_conf_pb2'
  # @@protoc_insertion_point(class_scope:apollo.cyber.proto.ShmConf)
  ))
_sym_db.RegisterMessage(ShmConf)

RtpsParticipantAttr = _reflection.GeneratedProtocolMessageType('RtpsParticipantAttr', (_message.Message,), dict(
  DESCRIPTOR = _RTPSPARTICIPANTATTR,
  __module__ = 'cyber.proto.transport_conf_pb2'
  # @@protoc_insertion_point(class_scope:apollo.cyber.proto.RtpsParticipantAttr)
  ))
_sym_db.RegisterMessage(RtpsParticipantAttr)

CommunicationMode = _reflection.GeneratedProtocolMessageType('CommunicationMode', (_message.Message,), dict(
  DESCRIPTOR = _COMMUNICATIONMODE,
  __module__ = 'cyber.proto.transport_conf_pb2'
  # @@protoc_insertion_point(class_scope:apollo.cyber.proto.CommunicationMode)
  ))
_sym_db.RegisterMessage(CommunicationMode)

ResourceLimit = _reflection.GeneratedProtocolMessageType('ResourceLimit', (_message.Message,), dict(
  DESCRIPTOR = _RESOURCELIMIT,
  __module__ = 'cyber.proto.transport_conf_pb2'
  # @@protoc_insertion_point(class_scope:apollo.cyber.proto.ResourceLimit)
  ))
_sym_db.RegisterMessage(ResourceLimit)

TransportConf = _reflection.GeneratedProtocolMessageType('TransportConf', (_message.Message,), dict(
  DESCRIPTOR = _TRANSPORTCONF,
  __module__ = 'cyber.proto.transport_conf_pb2'
  # @@protoc_insertion_point(class_scope:apollo.cyber.proto.TransportConf)
  ))
_sym_db.RegisterMessage(TransportConf)


# @@protoc_insertion_point(module_scope)
