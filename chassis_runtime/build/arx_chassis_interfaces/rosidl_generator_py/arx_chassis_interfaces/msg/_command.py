# generated from rosidl_generator_py/resource/_idl.py.em
# with input from arx_chassis_interfaces:msg/Command.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'sdk'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Command(type):
    """Metaclass of message 'Command'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('arx_chassis_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'arx_chassis_interfaces.msg.Command')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Command(metaclass=Metaclass_Command):
    """Message class 'Command'."""

    __slots__ = [
        '_request_id',
        '_scheduler_instance',
        '_client_id',
        '_operation',
        '_kind',
        '_source_id',
        '_task_id',
        '_generation',
        '_seq',
        '_issued_at',
        '_ttl_s',
        '_sdk',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'request_id': 'string',
        'scheduler_instance': 'string',
        'client_id': 'string',
        'operation': 'string',
        'kind': 'string',
        'source_id': 'string',
        'task_id': 'string',
        'generation': 'uint64',
        'seq': 'uint64',
        'issued_at': 'double',
        'ttl_s': 'double',
        'sdk': 'double[3]',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.request_id = kwargs.get('request_id', str())
        self.scheduler_instance = kwargs.get('scheduler_instance', str())
        self.client_id = kwargs.get('client_id', str())
        self.operation = kwargs.get('operation', str())
        self.kind = kwargs.get('kind', str())
        self.source_id = kwargs.get('source_id', str())
        self.task_id = kwargs.get('task_id', str())
        self.generation = kwargs.get('generation', int())
        self.seq = kwargs.get('seq', int())
        self.issued_at = kwargs.get('issued_at', float())
        self.ttl_s = kwargs.get('ttl_s', float())
        if 'sdk' not in kwargs:
            self.sdk = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.sdk = kwargs.get('sdk')

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.request_id != other.request_id:
            return False
        if self.scheduler_instance != other.scheduler_instance:
            return False
        if self.client_id != other.client_id:
            return False
        if self.operation != other.operation:
            return False
        if self.kind != other.kind:
            return False
        if self.source_id != other.source_id:
            return False
        if self.task_id != other.task_id:
            return False
        if self.generation != other.generation:
            return False
        if self.seq != other.seq:
            return False
        if self.issued_at != other.issued_at:
            return False
        if self.ttl_s != other.ttl_s:
            return False
        if any(self.sdk != other.sdk):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def request_id(self):
        """Message field 'request_id'."""
        return self._request_id

    @request_id.setter
    def request_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'request_id' field must be of type 'str'"
        self._request_id = value

    @builtins.property
    def scheduler_instance(self):
        """Message field 'scheduler_instance'."""
        return self._scheduler_instance

    @scheduler_instance.setter
    def scheduler_instance(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'scheduler_instance' field must be of type 'str'"
        self._scheduler_instance = value

    @builtins.property
    def client_id(self):
        """Message field 'client_id'."""
        return self._client_id

    @client_id.setter
    def client_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'client_id' field must be of type 'str'"
        self._client_id = value

    @builtins.property
    def operation(self):
        """Message field 'operation'."""
        return self._operation

    @operation.setter
    def operation(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'operation' field must be of type 'str'"
        self._operation = value

    @builtins.property
    def kind(self):
        """Message field 'kind'."""
        return self._kind

    @kind.setter
    def kind(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'kind' field must be of type 'str'"
        self._kind = value

    @builtins.property
    def source_id(self):
        """Message field 'source_id'."""
        return self._source_id

    @source_id.setter
    def source_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'source_id' field must be of type 'str'"
        self._source_id = value

    @builtins.property
    def task_id(self):
        """Message field 'task_id'."""
        return self._task_id

    @task_id.setter
    def task_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'task_id' field must be of type 'str'"
        self._task_id = value

    @builtins.property
    def generation(self):
        """Message field 'generation'."""
        return self._generation

    @generation.setter
    def generation(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'generation' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'generation' field must be an unsigned integer in [0, 18446744073709551615]"
        self._generation = value

    @builtins.property
    def seq(self):
        """Message field 'seq'."""
        return self._seq

    @seq.setter
    def seq(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'seq' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'seq' field must be an unsigned integer in [0, 18446744073709551615]"
        self._seq = value

    @builtins.property
    def issued_at(self):
        """Message field 'issued_at'."""
        return self._issued_at

    @issued_at.setter
    def issued_at(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'issued_at' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'issued_at' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._issued_at = value

    @builtins.property
    def ttl_s(self):
        """Message field 'ttl_s'."""
        return self._ttl_s

    @ttl_s.setter
    def ttl_s(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'ttl_s' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ttl_s' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ttl_s = value

    @builtins.property
    def sdk(self):
        """Message field 'sdk'."""
        return self._sdk

    @sdk.setter
    def sdk(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'sdk' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'sdk' numpy.ndarray() must have a size of 3"
                self._sdk = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'sdk' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._sdk = numpy.array(value, dtype=numpy.float64)
