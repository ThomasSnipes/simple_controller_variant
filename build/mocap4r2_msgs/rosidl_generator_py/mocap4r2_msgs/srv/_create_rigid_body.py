# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mocap4r2_msgs:srv/CreateRigidBody.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'markers'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CreateRigidBody_Request(type):
    """Metaclass of message 'CreateRigidBody_Request'."""

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
            module = import_type_support('mocap4r2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mocap4r2_msgs.srv.CreateRigidBody_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__create_rigid_body__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__create_rigid_body__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__create_rigid_body__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__create_rigid_body__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__create_rigid_body__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CreateRigidBody_Request(metaclass=Metaclass_CreateRigidBody_Request):
    """Message class 'CreateRigidBody_Request'."""

    __slots__ = [
        '_rigid_body_name',
        '_link_parent',
        '_markers',
    ]

    _fields_and_field_types = {
        'rigid_body_name': 'string',
        'link_parent': 'string',
        'markers': 'sequence<int32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.rigid_body_name = kwargs.get('rigid_body_name', str())
        self.link_parent = kwargs.get('link_parent', str())
        self.markers = array.array('i', kwargs.get('markers', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.rigid_body_name != other.rigid_body_name:
            return False
        if self.link_parent != other.link_parent:
            return False
        if self.markers != other.markers:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def rigid_body_name(self):
        """Message field 'rigid_body_name'."""
        return self._rigid_body_name

    @rigid_body_name.setter
    def rigid_body_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'rigid_body_name' field must be of type 'str'"
        self._rigid_body_name = value

    @builtins.property
    def link_parent(self):
        """Message field 'link_parent'."""
        return self._link_parent

    @link_parent.setter
    def link_parent(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'link_parent' field must be of type 'str'"
        self._link_parent = value

    @builtins.property
    def markers(self):
        """Message field 'markers'."""
        return self._markers

    @markers.setter
    def markers(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'markers' array.array() must have the type code of 'i'"
            self._markers = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'markers' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._markers = array.array('i', value)


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CreateRigidBody_Response(type):
    """Metaclass of message 'CreateRigidBody_Response'."""

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
            module = import_type_support('mocap4r2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mocap4r2_msgs.srv.CreateRigidBody_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__create_rigid_body__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__create_rigid_body__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__create_rigid_body__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__create_rigid_body__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__create_rigid_body__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CreateRigidBody_Response(metaclass=Metaclass_CreateRigidBody_Response):
    """Message class 'CreateRigidBody_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
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
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_CreateRigidBody(type):
    """Metaclass of service 'CreateRigidBody'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mocap4r2_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mocap4r2_msgs.srv.CreateRigidBody')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__create_rigid_body

            from mocap4r2_msgs.srv import _create_rigid_body
            if _create_rigid_body.Metaclass_CreateRigidBody_Request._TYPE_SUPPORT is None:
                _create_rigid_body.Metaclass_CreateRigidBody_Request.__import_type_support__()
            if _create_rigid_body.Metaclass_CreateRigidBody_Response._TYPE_SUPPORT is None:
                _create_rigid_body.Metaclass_CreateRigidBody_Response.__import_type_support__()


class CreateRigidBody(metaclass=Metaclass_CreateRigidBody):
    from mocap4r2_msgs.srv._create_rigid_body import CreateRigidBody_Request as Request
    from mocap4r2_msgs.srv._create_rigid_body import CreateRigidBody_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
