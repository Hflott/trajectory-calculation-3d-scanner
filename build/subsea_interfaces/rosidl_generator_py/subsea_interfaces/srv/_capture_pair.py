# generated from rosidl_generator_py/resource/_idl.py.em
# with input from subsea_interfaces:srv/CapturePair.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CapturePair_Request(type):
    """Metaclass of message 'CapturePair_Request'."""

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
            module = import_type_support('subsea_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'subsea_interfaces.srv.CapturePair_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__capture_pair__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__capture_pair__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__capture_pair__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__capture_pair__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__capture_pair__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CapturePair_Request(metaclass=Metaclass_CapturePair_Request):
    """Message class 'CapturePair_Request'."""

    __slots__ = [
        '_session_id',
        '_output_dir',
        '_jpeg_quality',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'session_id': 'string',
        'output_dir': 'string',
        'jpeg_quality': 'int32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
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
        self.session_id = kwargs.get('session_id', str())
        self.output_dir = kwargs.get('output_dir', str())
        self.jpeg_quality = kwargs.get('jpeg_quality', int())

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
        if self.session_id != other.session_id:
            return False
        if self.output_dir != other.output_dir:
            return False
        if self.jpeg_quality != other.jpeg_quality:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def session_id(self):
        """Message field 'session_id'."""
        return self._session_id

    @session_id.setter
    def session_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'session_id' field must be of type 'str'"
        self._session_id = value

    @builtins.property
    def output_dir(self):
        """Message field 'output_dir'."""
        return self._output_dir

    @output_dir.setter
    def output_dir(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'output_dir' field must be of type 'str'"
        self._output_dir = value

    @builtins.property
    def jpeg_quality(self):
        """Message field 'jpeg_quality'."""
        return self._jpeg_quality

    @jpeg_quality.setter
    def jpeg_quality(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'jpeg_quality' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'jpeg_quality' field must be an integer in [-2147483648, 2147483647]"
        self._jpeg_quality = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CapturePair_Response(type):
    """Metaclass of message 'CapturePair_Response'."""

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
            module = import_type_support('subsea_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'subsea_interfaces.srv.CapturePair_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__capture_pair__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__capture_pair__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__capture_pair__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__capture_pair__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__capture_pair__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CapturePair_Response(metaclass=Metaclass_CapturePair_Response):
    """Message class 'CapturePair_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_cam0_path',
        '_cam1_path',
        '_stamp',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'cam0_path': 'string',
        'cam1_path': 'string',
        'stamp': 'builtin_interfaces/Time',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
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
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.cam0_path = kwargs.get('cam0_path', str())
        self.cam1_path = kwargs.get('cam1_path', str())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        if self.cam0_path != other.cam0_path:
            return False
        if self.cam1_path != other.cam1_path:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value

    @builtins.property
    def cam0_path(self):
        """Message field 'cam0_path'."""
        return self._cam0_path

    @cam0_path.setter
    def cam0_path(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'cam0_path' field must be of type 'str'"
        self._cam0_path = value

    @builtins.property
    def cam1_path(self):
        """Message field 'cam1_path'."""
        return self._cam1_path

    @cam1_path.setter
    def cam1_path(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'cam1_path' field must be of type 'str'"
        self._cam1_path = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if self._check_fields:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CapturePair_Event(type):
    """Metaclass of message 'CapturePair_Event'."""

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
            module = import_type_support('subsea_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'subsea_interfaces.srv.CapturePair_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__capture_pair__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__capture_pair__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__capture_pair__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__capture_pair__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__capture_pair__event

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


class CapturePair_Event(metaclass=Metaclass_CapturePair_Event):
    """Message class 'CapturePair_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<subsea_interfaces/CapturePair_Request, 1>',
        'response': 'sequence<subsea_interfaces/CapturePair_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['subsea_interfaces', 'srv'], 'CapturePair_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['subsea_interfaces', 'srv'], 'CapturePair_Response'), 1),  # noqa: E501
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
            from subsea_interfaces.srv import CapturePair_Request
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
                 all(isinstance(v, CapturePair_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'CapturePair_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from subsea_interfaces.srv import CapturePair_Response
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
                 all(isinstance(v, CapturePair_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'CapturePair_Response'"
        self._response = value


class Metaclass_CapturePair(type):
    """Metaclass of service 'CapturePair'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('subsea_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'subsea_interfaces.srv.CapturePair')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__capture_pair

            from subsea_interfaces.srv import _capture_pair
            if _capture_pair.Metaclass_CapturePair_Request._TYPE_SUPPORT is None:
                _capture_pair.Metaclass_CapturePair_Request.__import_type_support__()
            if _capture_pair.Metaclass_CapturePair_Response._TYPE_SUPPORT is None:
                _capture_pair.Metaclass_CapturePair_Response.__import_type_support__()
            if _capture_pair.Metaclass_CapturePair_Event._TYPE_SUPPORT is None:
                _capture_pair.Metaclass_CapturePair_Event.__import_type_support__()


class CapturePair(metaclass=Metaclass_CapturePair):
    from subsea_interfaces.srv._capture_pair import CapturePair_Request as Request
    from subsea_interfaces.srv._capture_pair import CapturePair_Response as Response
    from subsea_interfaces.srv._capture_pair import CapturePair_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
