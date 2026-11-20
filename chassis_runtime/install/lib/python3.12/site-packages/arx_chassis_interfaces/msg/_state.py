# generated from rosidl_generator_py/resource/_idl.py.em
# with input from arx_chassis_interfaces:msg/State.idl
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
# Member 'applied_sdk'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_State(type):
    """Metaclass of message 'State'."""

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
                'arx_chassis_interfaces.msg.State')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class State(metaclass=Metaclass_State):
    """Message class 'State'."""

    __slots__ = [
        '_scheduler_instance',
        '_ready',
        '_driver_instance',
        '_active',
        '_client_id',
        '_source_id',
        '_task_id',
        '_generation',
        '_output_seq',
        '_expires_at',
        '_sdk',
        '_maintenance',
        '_applied_sdk',
        '_applied_seq',
        '_reason',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'scheduler_instance': 'string',
        'ready': 'boolean',
        'driver_instance': 'string',
        'active': 'boolean',
        'client_id': 'string',
        'source_id': 'string',
        'task_id': 'string',
        'generation': 'uint64',
        'output_seq': 'uint64',
        'expires_at': 'double',
        'sdk': 'double[3]',
        'maintenance': 'boolean',
        'applied_sdk': 'double[3]',
        'applied_seq': 'uint64',
        'reason': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
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
        self.scheduler_instance = kwargs.get('scheduler_instance', str())
        self.ready = kwargs.get('ready', bool())
        self.driver_instance = kwargs.get('driver_instance', str())
        self.active = kwargs.get('active', bool())
        self.client_id = kwargs.get('client_id', str())
        self.source_id = kwargs.get('source_id', str())
        self.task_id = kwargs.get('task_id', str())
        self.generation = kwargs.get('generation', int())
        self.output_seq = kwargs.get('output_seq', int())
        self.expires_at = kwargs.get('expires_at', float())
        if 'sdk' not in kwargs:
            self.sdk = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.sdk = kwargs.get('sdk')
        self.maintenance = kwargs.get('maintenance', bool())
        if 'applied_sdk' not in kwargs:
            self.applied_sdk = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.applied_sdk = kwargs.get('applied_sdk')
        self.applied_seq = kwargs.get('applied_seq', int())
        self.reason = kwargs.get('reason', str())

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
        if self.scheduler_instance != other.scheduler_instance:
            return False
        if self.ready != other.ready:
            return False
        if self.driver_instance != other.driver_instance:
            return False
        if self.active != other.active:
            return False
        if self.client_id != other.client_id:
            return False
        if self.source_id != other.source_id:
            return False
        if self.task_id != other.task_id:
            return False
        if self.generation != other.generation:
            return False
        if self.output_seq != other.output_seq:
            return False
        if self.expires_at != other.expires_at:
            return False
        if any(self.sdk != other.sdk):
            return False
        if self.maintenance != other.maintenance:
            return False
        if any(self.applied_sdk != other.applied_sdk):
            return False
        if self.applied_seq != other.applied_seq:
            return False
        if self.reason != other.reason:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
    def ready(self):
        """Message field 'ready'."""
        return self._ready

    @ready.setter
    def ready(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'ready' field must be of type 'bool'"
        self._ready = value

    @builtins.property
    def driver_instance(self):
        """Message field 'driver_instance'."""
        return self._driver_instance

    @driver_instance.setter
    def driver_instance(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'driver_instance' field must be of type 'str'"
        self._driver_instance = value

    @builtins.property
    def active(self):
        """Message field 'active'."""
        return self._active

    @active.setter
    def active(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'active' field must be of type 'bool'"
        self._active = value

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
    def output_seq(self):
        """Message field 'output_seq'."""
        return self._output_seq

    @output_seq.setter
    def output_seq(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'output_seq' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'output_seq' field must be an unsigned integer in [0, 18446744073709551615]"
        self._output_seq = value

    @builtins.property
    def expires_at(self):
        """Message field 'expires_at'."""
        return self._expires_at

    @expires_at.setter
    def expires_at(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'expires_at' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'expires_at' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._expires_at = value

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

    @builtins.property
    def maintenance(self):
        """Message field 'maintenance'."""
        return self._maintenance

    @maintenance.setter
    def maintenance(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'maintenance' field must be of type 'bool'"
        self._maintenance = value

    @builtins.property
    def applied_sdk(self):
        """Message field 'applied_sdk'."""
        return self._applied_sdk

    @applied_sdk.setter
    def applied_sdk(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'applied_sdk' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'applied_sdk' numpy.ndarray() must have a size of 3"
                self._applied_sdk = value
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
                "The 'applied_sdk' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._applied_sdk = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def applied_seq(self):
        """Message field 'applied_seq'."""
        return self._applied_seq

    @applied_seq.setter
    def applied_seq(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'applied_seq' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'applied_seq' field must be an unsigned integer in [0, 18446744073709551615]"
        self._applied_seq = value

    @builtins.property
    def reason(self):
        """Message field 'reason'."""
        return self._reason

    @reason.setter
    def reason(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'reason' field must be of type 'str'"
        self._reason = value
