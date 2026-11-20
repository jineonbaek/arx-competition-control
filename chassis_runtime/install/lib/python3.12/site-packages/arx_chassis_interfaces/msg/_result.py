# generated from rosidl_generator_py/resource/_idl.py.em
# with input from arx_chassis_interfaces:msg/Result.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Result(type):
    """Metaclass of message 'Result'."""

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
                'arx_chassis_interfaces.msg.Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__result
            cls._TYPE_SUPPORT = module.type_support_msg__msg__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__result

            from arx_chassis_interfaces.msg import State
            if State.__class__._TYPE_SUPPORT is None:
                State.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Result(metaclass=Metaclass_Result):
    """Message class 'Result'."""

    __slots__ = [
        '_request_id',
        '_accepted',
        '_reason',
        '_output_seq',
        '_state',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'request_id': 'string',
        'accepted': 'boolean',
        'reason': 'string',
        'output_seq': 'uint64',
        'state': 'arx_chassis_interfaces/State',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['arx_chassis_interfaces', 'msg'], 'State'),  # noqa: E501
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
        self.accepted = kwargs.get('accepted', bool())
        self.reason = kwargs.get('reason', str())
        self.output_seq = kwargs.get('output_seq', int())
        from arx_chassis_interfaces.msg import State
        self.state = kwargs.get('state', State())

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
        if self.accepted != other.accepted:
            return False
        if self.reason != other.reason:
            return False
        if self.output_seq != other.output_seq:
            return False
        if self.state != other.state:
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
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

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
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if self._check_fields:
            from arx_chassis_interfaces.msg import State
            assert \
                isinstance(value, State), \
                "The 'state' field must be a sub message of type 'State'"
        self._state = value
