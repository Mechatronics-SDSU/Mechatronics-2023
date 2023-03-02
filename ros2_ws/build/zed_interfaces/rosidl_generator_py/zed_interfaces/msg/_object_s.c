// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from zed_interfaces:msg/Object.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "zed_interfaces/msg/detail/object__struct.h"
#include "zed_interfaces/msg/detail/object__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool zed_interfaces__msg__bounding_box2_di__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * zed_interfaces__msg__bounding_box2_di__convert_to_py(void * raw_ros_message);
bool zed_interfaces__msg__bounding_box3_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * zed_interfaces__msg__bounding_box3_d__convert_to_py(void * raw_ros_message);
bool zed_interfaces__msg__bounding_box2_df__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * zed_interfaces__msg__bounding_box2_df__convert_to_py(void * raw_ros_message);
bool zed_interfaces__msg__bounding_box3_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * zed_interfaces__msg__bounding_box3_d__convert_to_py(void * raw_ros_message);
bool zed_interfaces__msg__skeleton2_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * zed_interfaces__msg__skeleton2_d__convert_to_py(void * raw_ros_message);
bool zed_interfaces__msg__skeleton3_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * zed_interfaces__msg__skeleton3_d__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool zed_interfaces__msg__object__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[34];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("zed_interfaces.msg._object.Object", full_classname_dest, 33) == 0);
  }
  zed_interfaces__msg__Object * ros_message = _ros_message;
  {  // label
    PyObject * field = PyObject_GetAttrString(_pymsg, "label");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->label, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // label_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "label_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->label_id = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // sublabel
    PyObject * field = PyObject_GetAttrString(_pymsg, "sublabel");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->sublabel, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // confidence
    PyObject * field = PyObject_GetAttrString(_pymsg, "confidence");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->confidence = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // position
    PyObject * field = PyObject_GetAttrString(_pymsg, "position");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
      Py_ssize_t size = 3;
      float * dest = ros_message->position;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // position_covariance
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_covariance");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
      Py_ssize_t size = 6;
      float * dest = ros_message->position_covariance;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
      Py_ssize_t size = 3;
      float * dest = ros_message->velocity;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // tracking_available
    PyObject * field = PyObject_GetAttrString(_pymsg, "tracking_available");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->tracking_available = (Py_True == field);
    Py_DECREF(field);
  }
  {  // tracking_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "tracking_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->tracking_state = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // action_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "action_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->action_state = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // bounding_box_2d
    PyObject * field = PyObject_GetAttrString(_pymsg, "bounding_box_2d");
    if (!field) {
      return false;
    }
    if (!zed_interfaces__msg__bounding_box2_di__convert_from_py(field, &ros_message->bounding_box_2d)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // bounding_box_3d
    PyObject * field = PyObject_GetAttrString(_pymsg, "bounding_box_3d");
    if (!field) {
      return false;
    }
    if (!zed_interfaces__msg__bounding_box3_d__convert_from_py(field, &ros_message->bounding_box_3d)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // dimensions_3d
    PyObject * field = PyObject_GetAttrString(_pymsg, "dimensions_3d");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
      Py_ssize_t size = 3;
      float * dest = ros_message->dimensions_3d;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // skeleton_available
    PyObject * field = PyObject_GetAttrString(_pymsg, "skeleton_available");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->skeleton_available = (Py_True == field);
    Py_DECREF(field);
  }
  {  // body_format
    PyObject * field = PyObject_GetAttrString(_pymsg, "body_format");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->body_format = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // head_bounding_box_2d
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_bounding_box_2d");
    if (!field) {
      return false;
    }
    if (!zed_interfaces__msg__bounding_box2_df__convert_from_py(field, &ros_message->head_bounding_box_2d)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // head_bounding_box_3d
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_bounding_box_3d");
    if (!field) {
      return false;
    }
    if (!zed_interfaces__msg__bounding_box3_d__convert_from_py(field, &ros_message->head_bounding_box_3d)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // head_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "head_position");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
      Py_ssize_t size = 3;
      float * dest = ros_message->head_position;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // skeleton_2d
    PyObject * field = PyObject_GetAttrString(_pymsg, "skeleton_2d");
    if (!field) {
      return false;
    }
    if (!zed_interfaces__msg__skeleton2_d__convert_from_py(field, &ros_message->skeleton_2d)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // skeleton_3d
    PyObject * field = PyObject_GetAttrString(_pymsg, "skeleton_3d");
    if (!field) {
      return false;
    }
    if (!zed_interfaces__msg__skeleton3_d__convert_from_py(field, &ros_message->skeleton_3d)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * zed_interfaces__msg__object__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Object */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("zed_interfaces.msg._object");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Object");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  zed_interfaces__msg__Object * ros_message = (zed_interfaces__msg__Object *)raw_ros_message;
  {  // label
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->label.data,
      strlen(ros_message->label.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "label", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // label_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->label_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "label_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sublabel
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->sublabel.data,
      strlen(ros_message->sublabel.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "sublabel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // confidence
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->confidence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confidence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "position");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
    assert(sizeof(npy_float32) == sizeof(float));
    npy_float32 * dst = (npy_float32 *)PyArray_GETPTR1(seq_field, 0);
    float * src = &(ros_message->position[0]);
    memcpy(dst, src, 3 * sizeof(float));
    Py_DECREF(field);
  }
  {  // position_covariance
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "position_covariance");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
    assert(sizeof(npy_float32) == sizeof(float));
    npy_float32 * dst = (npy_float32 *)PyArray_GETPTR1(seq_field, 0);
    float * src = &(ros_message->position_covariance[0]);
    memcpy(dst, src, 6 * sizeof(float));
    Py_DECREF(field);
  }
  {  // velocity
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "velocity");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
    assert(sizeof(npy_float32) == sizeof(float));
    npy_float32 * dst = (npy_float32 *)PyArray_GETPTR1(seq_field, 0);
    float * src = &(ros_message->velocity[0]);
    memcpy(dst, src, 3 * sizeof(float));
    Py_DECREF(field);
  }
  {  // tracking_available
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->tracking_available ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tracking_available", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tracking_state
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->tracking_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tracking_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // action_state
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->action_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "action_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bounding_box_2d
    PyObject * field = NULL;
    field = zed_interfaces__msg__bounding_box2_di__convert_to_py(&ros_message->bounding_box_2d);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "bounding_box_2d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bounding_box_3d
    PyObject * field = NULL;
    field = zed_interfaces__msg__bounding_box3_d__convert_to_py(&ros_message->bounding_box_3d);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "bounding_box_3d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dimensions_3d
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "dimensions_3d");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
    assert(sizeof(npy_float32) == sizeof(float));
    npy_float32 * dst = (npy_float32 *)PyArray_GETPTR1(seq_field, 0);
    float * src = &(ros_message->dimensions_3d[0]);
    memcpy(dst, src, 3 * sizeof(float));
    Py_DECREF(field);
  }
  {  // skeleton_available
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->skeleton_available ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "skeleton_available", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // body_format
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->body_format);
    {
      int rc = PyObject_SetAttrString(_pymessage, "body_format", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_bounding_box_2d
    PyObject * field = NULL;
    field = zed_interfaces__msg__bounding_box2_df__convert_to_py(&ros_message->head_bounding_box_2d);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "head_bounding_box_2d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_bounding_box_3d
    PyObject * field = NULL;
    field = zed_interfaces__msg__bounding_box3_d__convert_to_py(&ros_message->head_bounding_box_3d);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "head_bounding_box_3d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // head_position
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "head_position");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
    assert(sizeof(npy_float32) == sizeof(float));
    npy_float32 * dst = (npy_float32 *)PyArray_GETPTR1(seq_field, 0);
    float * src = &(ros_message->head_position[0]);
    memcpy(dst, src, 3 * sizeof(float));
    Py_DECREF(field);
  }
  {  // skeleton_2d
    PyObject * field = NULL;
    field = zed_interfaces__msg__skeleton2_d__convert_to_py(&ros_message->skeleton_2d);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "skeleton_2d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // skeleton_3d
    PyObject * field = NULL;
    field = zed_interfaces__msg__skeleton3_d__convert_to_py(&ros_message->skeleton_3d);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "skeleton_3d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
