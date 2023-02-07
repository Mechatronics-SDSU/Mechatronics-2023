# generated from rosidl_generator_py/resource/_idl.py.em
# with input from zed_interfaces:msg/PlaneStamped.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'extents'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PlaneStamped(type):
    """Metaclass of message 'PlaneStamped'."""

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
                'zed_interfaces.msg.PlaneStamped')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__plane_stamped
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__plane_stamped
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__plane_stamped
            cls._TYPE_SUPPORT = module.type_support_msg__msg__plane_stamped
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__plane_stamped

            from geometry_msgs.msg import Point32
            if Point32.__class__._TYPE_SUPPORT is None:
                Point32.__class__.__import_type_support__()

            from geometry_msgs.msg import Polygon
            if Polygon.__class__._TYPE_SUPPORT is None:
                Polygon.__class__.__import_type_support__()

            from geometry_msgs.msg import Transform
            if Transform.__class__._TYPE_SUPPORT is None:
                Transform.__class__.__import_type_support__()

            from shape_msgs.msg import Mesh
            if Mesh.__class__._TYPE_SUPPORT is None:
                Mesh.__class__.__import_type_support__()

            from shape_msgs.msg import Plane
            if Plane.__class__._TYPE_SUPPORT is None:
                Plane.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlaneStamped(metaclass=Metaclass_PlaneStamped):
    """Message class 'PlaneStamped'."""

    __slots__ = [
        '_header',
        '_mesh',
        '_coefficients',
        '_normal',
        '_center',
        '_pose',
        '_extents',
        '_bounds',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'mesh': 'shape_msgs/Mesh',
        'coefficients': 'shape_msgs/Plane',
        'normal': 'geometry_msgs/Point32',
        'center': 'geometry_msgs/Point32',
        'pose': 'geometry_msgs/Transform',
        'extents': 'float[2]',
        'bounds': 'geometry_msgs/Polygon',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['shape_msgs', 'msg'], 'Mesh'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['shape_msgs', 'msg'], 'Plane'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Transform'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 2),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Polygon'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from shape_msgs.msg import Mesh
        self.mesh = kwargs.get('mesh', Mesh())
        from shape_msgs.msg import Plane
        self.coefficients = kwargs.get('coefficients', Plane())
        from geometry_msgs.msg import Point32
        self.normal = kwargs.get('normal', Point32())
        from geometry_msgs.msg import Point32
        self.center = kwargs.get('center', Point32())
        from geometry_msgs.msg import Transform
        self.pose = kwargs.get('pose', Transform())
        if 'extents' not in kwargs:
            self.extents = numpy.zeros(2, dtype=numpy.float32)
        else:
            self.extents = numpy.array(kwargs.get('extents'), dtype=numpy.float32)
            assert self.extents.shape == (2, )
        from geometry_msgs.msg import Polygon
        self.bounds = kwargs.get('bounds', Polygon())

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
        if self.header != other.header:
            return False
        if self.mesh != other.mesh:
            return False
        if self.coefficients != other.coefficients:
            return False
        if self.normal != other.normal:
            return False
        if self.center != other.center:
            return False
        if self.pose != other.pose:
            return False
        if all(self.extents != other.extents):
            return False
        if self.bounds != other.bounds:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def mesh(self):
        """Message field 'mesh'."""
        return self._mesh

    @mesh.setter
    def mesh(self, value):
        if __debug__:
            from shape_msgs.msg import Mesh
            assert \
                isinstance(value, Mesh), \
                "The 'mesh' field must be a sub message of type 'Mesh'"
        self._mesh = value

    @property
    def coefficients(self):
        """Message field 'coefficients'."""
        return self._coefficients

    @coefficients.setter
    def coefficients(self, value):
        if __debug__:
            from shape_msgs.msg import Plane
            assert \
                isinstance(value, Plane), \
                "The 'coefficients' field must be a sub message of type 'Plane'"
        self._coefficients = value

    @property
    def normal(self):
        """Message field 'normal'."""
        return self._normal

    @normal.setter
    def normal(self, value):
        if __debug__:
            from geometry_msgs.msg import Point32
            assert \
                isinstance(value, Point32), \
                "The 'normal' field must be a sub message of type 'Point32'"
        self._normal = value

    @property
    def center(self):
        """Message field 'center'."""
        return self._center

    @center.setter
    def center(self, value):
        if __debug__:
            from geometry_msgs.msg import Point32
            assert \
                isinstance(value, Point32), \
                "The 'center' field must be a sub message of type 'Point32'"
        self._center = value

    @property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Transform
            assert \
                isinstance(value, Transform), \
                "The 'pose' field must be a sub message of type 'Transform'"
        self._pose = value

    @property
    def extents(self):
        """Message field 'extents'."""
        return self._extents

    @extents.setter
    def extents(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'extents' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 2, \
                "The 'extents' numpy.ndarray() must have a size of 2"
            self._extents = value
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
                 len(value) == 2 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'extents' field must be a set or sequence with length 2 and each value of type 'float'"
        self._extents = numpy.array(value, dtype=numpy.float32)

    @property
    def bounds(self):
        """Message field 'bounds'."""
        return self._bounds

    @bounds.setter
    def bounds(self, value):
        if __debug__:
            from geometry_msgs.msg import Polygon
            assert \
                isinstance(value, Polygon), \
                "The 'bounds' field must be a sub message of type 'Polygon'"
        self._bounds = value
