# generated from rosidl_generator_py/resource/_idl.py.em
# with input from arx_chassis_interfaces:srv/Manage.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Manage_Request(type):
    """Metaclass of message 'Manage_Request'."""

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
                'arx_chassis_interfaces.srv.Manage_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__manage__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__manage__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__manage__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__manage__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__manage__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Manage_Request(metaclass=Metaclass_Manage_Request):
    """Message class 'Manage_Request'."""

    __slots__ = [
        '_operation',
        '_scheduler_instance',
        '_client_id',
        '_profile',
        '_resume_key',
        '_output_seq',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'operation': 'string',
        'scheduler_instance': 'string',
        'client_id': 'string',
        'profile': 'string',
        'resume_key': 'string',
        'output_seq': 'uint64',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
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
        self.operation = kwargs.get('operation', str())
        self.scheduler_instance = kwargs.get('scheduler_instance', str())
        self.client_id = kwargs.get('client_id', str())
        self.profile = kwargs.get('profile', str())
        self.resume_key = kwargs.get('resume_key', str())
        self.output_seq = kwargs.get('output_seq', int())

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
        if self.operation != other.operation:
            return False
        if self.scheduler_instance != other.scheduler_instance:
            return False
        if self.client_id != other.client_id:
            return False
        if self.profile != other.profile:
            return False
        if self.resume_key != other.resume_key:
            return False
        if self.output_seq != other.output_seq:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
    def profile(self):
        """Message field 'profile'."""
        return self._profile

    @profile.setter
    def profile(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'profile' field must be of type 'str'"
        self._profile = value

    @builtins.property
    def resume_key(self):
        """Message field 'resume_key'."""
        return self._resume_key

    @resume_key.setter
    def resume_key(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'resume_key' field must be of type 'str'"
        self._resume_key = value

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Manage_Response(type):
    """Metaclass of message 'Manage_Response'."""

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
                'arx_chassis_interfaces.srv.Manage_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__manage__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__manage__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__manage__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__manage__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__manage__response

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


class Manage_Response(metaclass=Metaclass_Manage_Response):
    """Message class 'Manage_Response'."""

    __slots__ = [
        '_accepted',
        '_reason',
        '_client_id',
        '_state',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'reason': 'string',
        'client_id': 'string',
        'state': 'arx_chassis_interfaces/State',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
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
        self.accepted = kwargs.get('accepted', bool())
        self.reason = kwargs.get('reason', str())
        self.client_id = kwargs.get('client_id', str())
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
        if self.accepted != other.accepted:
            return False
        if self.reason != other.reason:
            return False
        if self.client_id != other.client_id:
            return False
        if self.state != other.state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Manage_Event(type):
    """Metaclass of message 'Manage_Event'."""

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
                'arx_chassis_interfaces.srv.Manage_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__manage__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__manage__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__manage__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__manage__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__manage__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Manage_Event(metaclass=Metaclass_Manage_Event):
    """Message class 'Manage_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<arx_chassis_interfaces/Manage_Request, 1>',
        'response': 'sequence<arx_chassis_interfaces/Manage_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['arx_chassis_interfaces', 'srv'], 'Manage_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['arx_chassis_interfaces', 'srv'], 'Manage_Response'), 1),  # noqa: E501
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
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

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
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from arx_chassis_interfaces.srv import Manage_Request
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
                 len(value) <= 1 and
                 all(isinstance(v, Manage_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'Manage_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from arx_chassis_interfaces.srv import Manage_Response
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
                 len(value) <= 1 and
                 all(isinstance(v, Manage_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'Manage_Response'"
        self._response = value


class Metaclass_Manage(type):
    """Metaclass of service 'Manage'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('arx_chassis_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'arx_chassis_interfaces.srv.Manage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__manage

            from arx_chassis_interfaces.srv import _manage
            if _manage.Metaclass_Manage_Request._TYPE_SUPPORT is None:
                _manage.Metaclass_Manage_Request.__import_type_support__()
            if _manage.Metaclass_Manage_Response._TYPE_SUPPORT is None:
                _manage.Metaclass_Manage_Response.__import_type_support__()
            if _manage.Metaclass_Manage_Event._TYPE_SUPPORT is None:
                _manage.Metaclass_Manage_Event.__import_type_support__()


class Manage(metaclass=Metaclass_Manage):
    from arx_chassis_interfaces.srv._manage import Manage_Request as Request
    from arx_chassis_interfaces.srv._manage import Manage_Response as Response
    from arx_chassis_interfaces.srv._manage import Manage_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
