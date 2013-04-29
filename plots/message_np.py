#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function
import struct

import numpy as np

class Message_np:
    def __init__(self, pb_module, message_name=None, filename=None):
        self.pb_module = __import__(pb_module)
        self.valid_cpptype_map = False
        self.cpptype_map = None
        self.cpp_to_npdtype_map = None
        self.message_npdtype_map = None

        self.filename = None
        self.messages_pb = None
        self.messages_np = None

        self._set_cpptype_map()
        self._set_cpp_to_npdtype_map()
        self._set_message_npdtype_map()
        
        if message_name and filename:
            self.load_messages_from_file(message_name, filename)

    def message_types(self):
        return self.message_npdtype_map.keys()

    def message_fields(self, message_name=None, message_type=None):
        if message_name:
            type_descriptor = getattr(self.pb_module, message_name).DESCRIPTOR
        elif message_type:
            try: # protobuf message
                type_descriptor = message_type.DESCRIPTOR
            except AttributeError: # npdtype message
                return message_type.fields.keys()
        else:
            type_descriptor = self.messages_pb[0].DESCRIPTOR
        return type_descriptor.fields_by_name.keys()

    def get_messages_pb(self):
        return self.messages_pb

    def get_messages_np(self):
        return self.messages_np

    def _set_cpptype_map(self):
        self.cpptype_map = dict([
            (1, 'CPPTYPE_INT32'),
            (2, 'CPPTYPE_INT64'),
            (3, 'CPPTYPE_UINT32'),
            (4, 'CPPTYPE_UINT64'),
            (5, 'CPPTYPE_DOUBLE'),
            (6, 'CPPTYPE_FLOAT'),
            (7, 'CPPTYPE_BOOL'),
            (8, 'CPPTYPE_ENUM'),
            (9, 'CPPTYPE_STRING'),
            (10, 'CPPTYPE_MESSAGE')])

    def _set_cpp_to_npdtype_map(self):
        self.cpp_to_npdtype_map = dict([
            ('CPPTYPE_INT32', np.int32),
            ('CPPTYPE_INT64', np.int64),
            ('CPPTYPE_UINT32', np.uint32),
            ('CPPTYPE_UINT64', np.uint64),
            ('CPPTYPE_DOUBLE', np.float64),
            ('CPPTYPE_FLOAT', np.float32),
            ('CPPTYPE_BOOL', np.bool),
            ('CPPTYPE_ENUM', np.int32),
            ('CPPTYPE_STRING', str)])

    def load_messages_from_file(self, message_name, filename,
                                byte_format='<H', message_size_bytes=None):
        message_type = getattr(self.pb_module, message_name)
        if message_size_bytes is None:
            message_size_bytes = struct.calcsize(byte_format)
        self.messages_pb = []
        self.filename = filename
        with open(filename, 'rb') as f:
            while True:
                size_bytes = f.read(message_size_bytes)
                if not size_bytes: # eof
                    break
                message_size = struct.unpack(byte_format, size_bytes)[0]
                m = message_type()
                m.ParseFromString(f.read(message_size))
                self.messages_pb.append(m)
        self.messages_np = np.empty((len(self.messages_pb),),
                                    dtype=self.message_npdtype_map[message_name])
        self._set_messages_np()
        return self.get_messages_np()

    def _convert_message_to_npdtype(self, message_pb, message_np):
        npdtype = self.message_npdtype_map[message_pb.__class__.__name__]
        for field in npdtype.fields.keys():
            subfields = npdtype[field].fields
            if subfields:
                self._convert_message_to_npdtype(getattr(message_pb, field),
                                                message_np[field])
            else:
                message_np[field] = getattr(message_pb, field)

    def _set_messages_np(self):
        for (message_pb, message_np) in zip(self.messages_pb, self.messages_np):
            self._convert_message_to_npdtype(message_pb, message_np)

    def _validate_cpptype_map(self, field):
        cpptypes = [type_ for type_ in dir(field) if type_.startswith('CPPTYPE')]
        for type_ in cpptypes:
            cpptype_value = getattr(field, type_)
            stored_cpptype = self.cpptype_map[cpptype_value]
            if stored_cpptype != type_:
                raise ValueError(("CPPTYPE_VALUE '{0}': stored name" +
                    "'{1}' does not match protobuf DESCRIPTOP name" +
                    "'{2}'").format(cpptype_value, stored_cpptype, type_))
    
    def _get_message_field_npdtypes(self, message_name):
        type_descriptor = getattr(self.pb_module, message_name).DESCRIPTOR
        field_type_map = {}
        has_dependency = False

        for field in type_descriptor.fields:
            if not self.valid_cpptype_map:
                self._validate_cpptype_map(field)

            if self.cpptype_map[field.cpp_type] == 'CPPTYPE_MESSAGE':
                field_type_map[field.name] = (field.cpp_type,
                                              field.message_type.name)
            else:
                npdtype = self.cpp_to_npdtype_map[self.cpptype_map[field.cpp_type]]
                field_type_map[field.name] = npdtype
                has_dependency = True
        return field_type_map, has_dependency
    
    def _set_message_npdtype_map(self):
        self.message_npdtype_map = {}
        message_names = self.pb_module.DESCRIPTOR.message_types_by_name.keys()
        message_fieldtypes = {}
    
        while message_names:
            message_name = message_names.pop(0)

            if message_name in message_fieldtypes:
                ft_map = message_fieldtypes[message_name]
                # second elem in tuple is name of message_type
                old_dependencies = [(field, type_) for field, type_ in ft_map.iteritems()
                                    if isinstance(type_, tuple)]
                has_dependency = False
                for field, type_ in old_dependencies:
                    _, dep_message_name = type_
                    if dep_message_name in self.message_npdtype_map:
                        ft_map[field] = self.message_npdtype_map[dep_message_name]
                    else: # message_type not yet defined
                        has_dependency = True
                        continue

            else:
                ft_map, has_dependency = self._get_message_field_npdtypes(message_name)

            if has_dependency:
                message_names.append(message_name)
                message_fieldtypes[message_name] = ft_map
            else:
                self.message_npdtype_map[message_name] = np.dtype(ft_map.items())


#if __name__ == '__main__':
#    samples_np = Message_np('sample.pb2')
#    samples_np.load_messages_from_file('samples.dat', 'Sample')
