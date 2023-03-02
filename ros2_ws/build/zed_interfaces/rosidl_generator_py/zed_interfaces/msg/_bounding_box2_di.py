# generated from rosidl_generator_py/resource/_idl.py.em
# with input from zed_interfaces:msg/BoundingBox2Di.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BoundingBox2Di(type):
    """Metaclass of message 'BoundingBox2Di'."""

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
            module = import_type_support('zed_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'zed_interfaces.msg.BoundingBox2Di')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bounding_box2_di
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bounding_box2_di
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bounding_box2_di
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bounding_box2_di
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bounding_box2_di

            from zed_interfaces.msg import Keypoint2Di
            if Keypoint2Di.__class__._TYPE_SUPPORT is None:
                Keypoint2Di.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BoundingBox2Di(metaclass=Metaclass_BoundingBox2Di):
    """Message class 'BoundingBox2Di'."""

    __slots__ = [
        '_corners',
    ]

    _fields_and_field_types = {
        'corners': 'zed_interfaces/Keypoint2Di[4]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['zed_interfaces', 'msg'], 'Keypoint2Di'), 4),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from zed_interfaces.msg import Keypoint2Di
        self.corners = kwargs.get(
            'corners',
            [Keypoint2Di() for x in range(4)]
        )

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
        if self.corners != other.corners:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def corners(self):
        """Message field 'corners'."""
        return self._corners

    @corners.setter
    def corners(self, value):
        if __debug__:
            from zed_interfaces.msg import Keypoint2Di
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
                 len(value) == 4 and
                 all(isinstance(v, Keypoint2Di) for v in value) and
                 True), \
                "The 'corners' field must be a set or sequence with length 4 and each value of type 'Keypoint2Di'"
        self._corners = value
