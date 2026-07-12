# generated from rosidl_generator_py/resource/_idl.py.em
# with input from multi_robot_interfaces:msg/PVAConstraints.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'a'
# Member 'b'
# Member 'c'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PVAConstraints(type):
    """Metaclass of message 'PVAConstraints'."""

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
            module = import_type_support('multi_robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'multi_robot_interfaces.msg.PVAConstraints')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pva_constraints
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pva_constraints
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pva_constraints
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pva_constraints
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pva_constraints

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PVAConstraints(metaclass=Metaclass_PVAConstraints):
    """Message class 'PVAConstraints'."""

    __slots__ = [
        '_robot_id',
        '_a',
        '_b',
        '_c',
        '_v_goal',
        '_w_goal',
        '_v_star',
        '_w_star',
        '_mode',
        '_search_dir',
    ]

    _fields_and_field_types = {
        'robot_id': 'string',
        'a': 'sequence<double>',
        'b': 'sequence<double>',
        'c': 'sequence<double>',
        'v_goal': 'double',
        'w_goal': 'double',
        'v_star': 'double',
        'w_star': 'double',
        'mode': 'int32',
        'search_dir': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_id = kwargs.get('robot_id', str())
        self.a = array.array('d', kwargs.get('a', []))
        self.b = array.array('d', kwargs.get('b', []))
        self.c = array.array('d', kwargs.get('c', []))
        self.v_goal = kwargs.get('v_goal', float())
        self.w_goal = kwargs.get('w_goal', float())
        self.v_star = kwargs.get('v_star', float())
        self.w_star = kwargs.get('w_star', float())
        self.mode = kwargs.get('mode', int())
        self.search_dir = kwargs.get('search_dir', int())

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
        if self.robot_id != other.robot_id:
            return False
        if self.a != other.a:
            return False
        if self.b != other.b:
            return False
        if self.c != other.c:
            return False
        if self.v_goal != other.v_goal:
            return False
        if self.w_goal != other.w_goal:
            return False
        if self.v_star != other.v_star:
            return False
        if self.w_star != other.w_star:
            return False
        if self.mode != other.mode:
            return False
        if self.search_dir != other.search_dir:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_id' field must be of type 'str'"
        self._robot_id = value

    @builtins.property
    def a(self):
        """Message field 'a'."""
        return self._a

    @a.setter
    def a(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'a' array.array() must have the type code of 'd'"
            self._a = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'a' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._a = array.array('d', value)

    @builtins.property
    def b(self):
        """Message field 'b'."""
        return self._b

    @b.setter
    def b(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'b' array.array() must have the type code of 'd'"
            self._b = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'b' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._b = array.array('d', value)

    @builtins.property
    def c(self):
        """Message field 'c'."""
        return self._c

    @c.setter
    def c(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'c' array.array() must have the type code of 'd'"
            self._c = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'c' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._c = array.array('d', value)

    @builtins.property
    def v_goal(self):
        """Message field 'v_goal'."""
        return self._v_goal

    @v_goal.setter
    def v_goal(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'v_goal' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'v_goal' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._v_goal = value

    @builtins.property
    def w_goal(self):
        """Message field 'w_goal'."""
        return self._w_goal

    @w_goal.setter
    def w_goal(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'w_goal' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'w_goal' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._w_goal = value

    @builtins.property
    def v_star(self):
        """Message field 'v_star'."""
        return self._v_star

    @v_star.setter
    def v_star(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'v_star' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'v_star' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._v_star = value

    @builtins.property
    def w_star(self):
        """Message field 'w_star'."""
        return self._w_star

    @w_star.setter
    def w_star(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'w_star' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'w_star' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._w_star = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'mode' field must be an integer in [-2147483648, 2147483647]"
        self._mode = value

    @builtins.property
    def search_dir(self):
        """Message field 'search_dir'."""
        return self._search_dir

    @search_dir.setter
    def search_dir(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'search_dir' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'search_dir' field must be an integer in [-2147483648, 2147483647]"
        self._search_dir = value
