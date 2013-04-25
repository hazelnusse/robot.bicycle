#!/usr/bin/env python2

from __future__ import print_function
import sys

import numpy as np


class Message_np:
    def __init__(self, pb_module):
        self.pb_module = __import__(pb_module)
        self.valid_cpptype_map = False
        self.set_cpptype_map()
        self.set_cpp_to_npdtype_map()
        self.create_message_npdtype()

    def set_cpptype_map(self):
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

    def set_cpp_to_npdtype_map(self):
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

    def validate_cpptype_map(self, field):
        cpptypes = [type for type in dir(field) if type.startswith('CPPTYPE')]
        self.cpptype_map = {}
        for type in cpptypes:
            self.cpptype_map[getattr(field, type)] = type
        self.valid_cpptype_map = True
    
    def get_message_field_npdtypes(self, message_name):
        type_descriptor = getattr(self.pb_module, message_name).DESCRIPTOR
        field_names = type_descriptor.fields_by_name.keys()
        field_type_map = {}
        has_dependency = False

        for field in type_descriptor.fields:
            # validate the cpptype_map first time we get message field info
            if not self.valid_cpptype_map:
                self.validate_cpptype_map(field)

            if self.cpptype_map[field.cpp_type] == 'CPPTYPE_MESSAGE':
                field_type_map[field.name] = (field.cpp_type,
                                              field.message_type.name)
            else:
                npdtype = self.cpp_to_npdtype_map[self.cpptype_map[field.cpp_type]]
                field_type_map[field.name] = npdtype
                has_dependency = True
        return field_type_map, has_dependency
    
    def create_message_npdtype(self):
        self.message_npdtype_map = {}
        message_types = self.pb_module.DESCRIPTOR.message_types_by_name.keys()
        message_fields = {}
    
        while message_types:
            message_name = message_types.pop(0)

            if message_name in message_fields:
                ft_map = message_fields[message_name]
                # second elem in tuple is name of message_type
                old_dependencies = [(field, type) for field, type in ft_map.iteritems()
                                    if isinstance(type, tuple)]
                has_dependency = False
                for field, type in old_dependencies:
                    _, dep_message_type = type
                    if dep_message_type in self.message_npdtype_map:
                        npdtype = self.message_npdtype_map[dep_message_type]
                    else: # message_type not yet defined
                        has_dependency = True
                        continue
                    ft_map[field] = npdtype

            else:
                ft_map, has_dependency = self.get_message_field_npdtypes(message_name)

            if has_dependency:
                message_types.append(message_name)
                message_fields[message_name] = ft_map
            else:
                self.message_npdtype_map[message_name] = np.dtype(ft_map.items())

if __name__ == '__main__':
    Message_np('Sample_pb2')
