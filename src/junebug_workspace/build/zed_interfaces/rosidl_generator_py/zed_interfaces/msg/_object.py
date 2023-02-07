# generated from rosidl_generator_py/resource/_idl.py.em
# with input from zed_interfaces:msg/Object.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'position'
# Member 'position_covariance'
# Member 'velocity'
# Member 'dimensions_3d'
# Member 'head_position'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Object(type):
    """Metaclass of message 'Object'."""

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
                'zed_interfaces.msg.Object')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__object
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__object
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__object
            cls._TYPE_SUPPORT = module.type_support_msg__msg__object
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__object

            from zed_interfaces.msg import BoundingBox2Df
            if BoundingBox2Df.__class__._TYPE_SUPPORT is None:
                BoundingBox2Df.__class__.__import_type_support__()

            from zed_interfaces.msg import BoundingBox2Di
            if BoundingBox2Di.__class__._TYPE_SUPPORT is None:
                BoundingBox2Di.__class__.__import_type_support__()

            from zed_interfaces.msg import BoundingBox3D
            if BoundingBox3D.__class__._TYPE_SUPPORT is None:
                BoundingBox3D.__class__.__import_type_support__()

            from zed_interfaces.msg import Skeleton2D
            if Skeleton2D.__class__._TYPE_SUPPORT is None:
                Skeleton2D.__class__.__import_type_support__()

            from zed_interfaces.msg import Skeleton3D
            if Skeleton3D.__class__._TYPE_SUPPORT is None:
                Skeleton3D.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Object(metaclass=Metaclass_Object):
    """Message class 'Object'."""

    __slots__ = [
        '_label',
        '_label_id',
        '_sublabel',
        '_confidence',
        '_position',
        '_position_covariance',
        '_velocity',
        '_tracking_available',
        '_tracking_state',
        '_action_state',
        '_bounding_box_2d',
        '_bounding_box_3d',
        '_dimensions_3d',
        '_skeleton_available',
        '_body_format',
        '_head_bounding_box_2d',
        '_head_bounding_box_3d',
        '_head_position',
        '_skeleton_2d',
        '_skeleton_3d',
    ]

    _fields_and_field_types = {
        'label': 'string',
        'label_id': 'int16',
        'sublabel': 'string',
        'confidence': 'float',
        'position': 'float[3]',
        'position_covariance': 'float[6]',
        'velocity': 'float[3]',
        'tracking_available': 'boolean',
        'tracking_state': 'int8',
        'action_state': 'int8',
        'bounding_box_2d': 'zed_interfaces/BoundingBox2Di',
        'bounding_box_3d': 'zed_interfaces/BoundingBox3D',
        'dimensions_3d': 'float[3]',
        'skeleton_available': 'boolean',
        'body_format': 'int8',
        'head_bounding_box_2d': 'zed_interfaces/BoundingBox2Df',
        'head_bounding_box_3d': 'zed_interfaces/BoundingBox3D',
        'head_position': 'float[3]',
        'skeleton_2d': 'zed_interfaces/Skeleton2D',
        'skeleton_3d': 'zed_interfaces/Skeleton3D',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['zed_interfaces', 'msg'], 'BoundingBox2Di'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['zed_interfaces', 'msg'], 'BoundingBox3D'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['zed_interfaces', 'msg'], 'BoundingBox2Df'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['zed_interfaces', 'msg'], 'BoundingBox3D'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 3),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['zed_interfaces', 'msg'], 'Skeleton2D'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['zed_interfaces', 'msg'], 'Skeleton3D'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.label = kwargs.get('label', str())
        self.label_id = kwargs.get('label_id', int())
        self.sublabel = kwargs.get('sublabel', str())
        self.confidence = kwargs.get('confidence', float())
        if 'position' not in kwargs:
            self.position = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.position = numpy.array(kwargs.get('position'), dtype=numpy.float32)
            assert self.position.shape == (3, )
        if 'position_covariance' not in kwargs:
            self.position_covariance = numpy.zeros(6, dtype=numpy.float32)
        else:
            self.position_covariance = numpy.array(kwargs.get('position_covariance'), dtype=numpy.float32)
            assert self.position_covariance.shape == (6, )
        if 'velocity' not in kwargs:
            self.velocity = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.velocity = numpy.array(kwargs.get('velocity'), dtype=numpy.float32)
            assert self.velocity.shape == (3, )
        self.tracking_available = kwargs.get('tracking_available', bool())
        self.tracking_state = kwargs.get('tracking_state', int())
        self.action_state = kwargs.get('action_state', int())
        from zed_interfaces.msg import BoundingBox2Di
        self.bounding_box_2d = kwargs.get('bounding_box_2d', BoundingBox2Di())
        from zed_interfaces.msg import BoundingBox3D
        self.bounding_box_3d = kwargs.get('bounding_box_3d', BoundingBox3D())
        if 'dimensions_3d' not in kwargs:
            self.dimensions_3d = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.dimensions_3d = numpy.array(kwargs.get('dimensions_3d'), dtype=numpy.float32)
            assert self.dimensions_3d.shape == (3, )
        self.skeleton_available = kwargs.get('skeleton_available', bool())
        self.body_format = kwargs.get('body_format', int())
        from zed_interfaces.msg import BoundingBox2Df
        self.head_bounding_box_2d = kwargs.get('head_bounding_box_2d', BoundingBox2Df())
        from zed_interfaces.msg import BoundingBox3D
        self.head_bounding_box_3d = kwargs.get('head_bounding_box_3d', BoundingBox3D())
        if 'head_position' not in kwargs:
            self.head_position = numpy.zeros(3, dtype=numpy.float32)
        else:
            self.head_position = numpy.array(kwargs.get('head_position'), dtype=numpy.float32)
            assert self.head_position.shape == (3, )
        from zed_interfaces.msg import Skeleton2D
        self.skeleton_2d = kwargs.get('skeleton_2d', Skeleton2D())
        from zed_interfaces.msg import Skeleton3D
        self.skeleton_3d = kwargs.get('skeleton_3d', Skeleton3D())

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
        if self.label != other.label:
            return False
        if self.label_id != other.label_id:
            return False
        if self.sublabel != other.sublabel:
            return False
        if self.confidence != other.confidence:
            return False
        if all(self.position != other.position):
            return False
        if all(self.position_covariance != other.position_covariance):
            return False
        if all(self.velocity != other.velocity):
            return False
        if self.tracking_available != other.tracking_available:
            return False
        if self.tracking_state != other.tracking_state:
            return False
        if self.action_state != other.action_state:
            return False
        if self.bounding_box_2d != other.bounding_box_2d:
            return False
        if self.bounding_box_3d != other.bounding_box_3d:
            return False
        if all(self.dimensions_3d != other.dimensions_3d):
            return False
        if self.skeleton_available != other.skeleton_available:
            return False
        if self.body_format != other.body_format:
            return False
        if self.head_bounding_box_2d != other.head_bounding_box_2d:
            return False
        if self.head_bounding_box_3d != other.head_bounding_box_3d:
            return False
        if all(self.head_position != other.head_position):
            return False
        if self.skeleton_2d != other.skeleton_2d:
            return False
        if self.skeleton_3d != other.skeleton_3d:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def label(self):
        """Message field 'label'."""
        return self._label

    @label.setter
    def label(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'label' field must be of type 'str'"
        self._label = value

    @property
    def label_id(self):
        """Message field 'label_id'."""
        return self._label_id

    @label_id.setter
    def label_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'label_id' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'label_id' field must be an integer in [-32768, 32767]"
        self._label_id = value

    @property
    def sublabel(self):
        """Message field 'sublabel'."""
        return self._sublabel

    @sublabel.setter
    def sublabel(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'sublabel' field must be of type 'str'"
        self._sublabel = value

    @property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
        self._confidence = value

    @property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'position' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'position' numpy.ndarray() must have a size of 3"
            self._position = value
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'position' field must be a set or sequence with length 3 and each value of type 'float'"
        self._position = numpy.array(value, dtype=numpy.float32)

    @property
    def position_covariance(self):
        """Message field 'position_covariance'."""
        return self._position_covariance

    @position_covariance.setter
    def position_covariance(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'position_covariance' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 6, \
                "The 'position_covariance' numpy.ndarray() must have a size of 6"
            self._position_covariance = value
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
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'position_covariance' field must be a set or sequence with length 6 and each value of type 'float'"
        self._position_covariance = numpy.array(value, dtype=numpy.float32)

    @property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'velocity' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'velocity' numpy.ndarray() must have a size of 3"
            self._velocity = value
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'velocity' field must be a set or sequence with length 3 and each value of type 'float'"
        self._velocity = numpy.array(value, dtype=numpy.float32)

    @property
    def tracking_available(self):
        """Message field 'tracking_available'."""
        return self._tracking_available

    @tracking_available.setter
    def tracking_available(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'tracking_available' field must be of type 'bool'"
        self._tracking_available = value

    @property
    def tracking_state(self):
        """Message field 'tracking_state'."""
        return self._tracking_state

    @tracking_state.setter
    def tracking_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'tracking_state' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'tracking_state' field must be an integer in [-128, 127]"
        self._tracking_state = value

    @property
    def action_state(self):
        """Message field 'action_state'."""
        return self._action_state

    @action_state.setter
    def action_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'action_state' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'action_state' field must be an integer in [-128, 127]"
        self._action_state = value

    @property
    def bounding_box_2d(self):
        """Message field 'bounding_box_2d'."""
        return self._bounding_box_2d

    @bounding_box_2d.setter
    def bounding_box_2d(self, value):
        if __debug__:
            from zed_interfaces.msg import BoundingBox2Di
            assert \
                isinstance(value, BoundingBox2Di), \
                "The 'bounding_box_2d' field must be a sub message of type 'BoundingBox2Di'"
        self._bounding_box_2d = value

    @property
    def bounding_box_3d(self):
        """Message field 'bounding_box_3d'."""
        return self._bounding_box_3d

    @bounding_box_3d.setter
    def bounding_box_3d(self, value):
        if __debug__:
            from zed_interfaces.msg import BoundingBox3D
            assert \
                isinstance(value, BoundingBox3D), \
                "The 'bounding_box_3d' field must be a sub message of type 'BoundingBox3D'"
        self._bounding_box_3d = value

    @property
    def dimensions_3d(self):
        """Message field 'dimensions_3d'."""
        return self._dimensions_3d

    @dimensions_3d.setter
    def dimensions_3d(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'dimensions_3d' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'dimensions_3d' numpy.ndarray() must have a size of 3"
            self._dimensions_3d = value
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'dimensions_3d' field must be a set or sequence with length 3 and each value of type 'float'"
        self._dimensions_3d = numpy.array(value, dtype=numpy.float32)

    @property
    def skeleton_available(self):
        """Message field 'skeleton_available'."""
        return self._skeleton_available

    @skeleton_available.setter
    def skeleton_available(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'skeleton_available' field must be of type 'bool'"
        self._skeleton_available = value

    @property
    def body_format(self):
        """Message field 'body_format'."""
        return self._body_format

    @body_format.setter
    def body_format(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'body_format' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'body_format' field must be an integer in [-128, 127]"
        self._body_format = value

    @property
    def head_bounding_box_2d(self):
        """Message field 'head_bounding_box_2d'."""
        return self._head_bounding_box_2d

    @head_bounding_box_2d.setter
    def head_bounding_box_2d(self, value):
        if __debug__:
            from zed_interfaces.msg import BoundingBox2Df
            assert \
                isinstance(value, BoundingBox2Df), \
                "The 'head_bounding_box_2d' field must be a sub message of type 'BoundingBox2Df'"
        self._head_bounding_box_2d = value

    @property
    def head_bounding_box_3d(self):
        """Message field 'head_bounding_box_3d'."""
        return self._head_bounding_box_3d

    @head_bounding_box_3d.setter
    def head_bounding_box_3d(self, value):
        if __debug__:
            from zed_interfaces.msg import BoundingBox3D
            assert \
                isinstance(value, BoundingBox3D), \
                "The 'head_bounding_box_3d' field must be a sub message of type 'BoundingBox3D'"
        self._head_bounding_box_3d = value

    @property
    def head_position(self):
        """Message field 'head_position'."""
        return self._head_position

    @head_position.setter
    def head_position(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'head_position' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 3, \
                "The 'head_position' numpy.ndarray() must have a size of 3"
            self._head_position = value
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'head_position' field must be a set or sequence with length 3 and each value of type 'float'"
        self._head_position = numpy.array(value, dtype=numpy.float32)

    @property
    def skeleton_2d(self):
        """Message field 'skeleton_2d'."""
        return self._skeleton_2d

    @skeleton_2d.setter
    def skeleton_2d(self, value):
        if __debug__:
            from zed_interfaces.msg import Skeleton2D
            assert \
                isinstance(value, Skeleton2D), \
                "The 'skeleton_2d' field must be a sub message of type 'Skeleton2D'"
        self._skeleton_2d = value

    @property
    def skeleton_3d(self):
        """Message field 'skeleton_3d'."""
        return self._skeleton_3d

    @skeleton_3d.setter
    def skeleton_3d(self, value):
        if __debug__:
            from zed_interfaces.msg import Skeleton3D
            assert \
                isinstance(value, Skeleton3D), \
                "The 'skeleton_3d' field must be a sub message of type 'Skeleton3D'"
        self._skeleton_3d = value
