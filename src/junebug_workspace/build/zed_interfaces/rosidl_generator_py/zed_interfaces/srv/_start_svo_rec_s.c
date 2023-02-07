// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from zed_interfaces:srv/StartSvoRec.idl
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
#include "zed_interfaces/srv/detail/start_svo_rec__struct.h"
#include "zed_interfaces/srv/detail/start_svo_rec__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool zed_interfaces__srv__start_svo_rec__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[54];
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
    assert(strncmp("zed_interfaces.srv._start_svo_rec.StartSvoRec_Request", full_classname_dest, 53) == 0);
  }
  zed_interfaces__srv__StartSvoRec_Request * ros_message = _ros_message;
  {  // bitrate
    PyObject * field = PyObject_GetAttrString(_pymsg, "bitrate");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bitrate = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // compression_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "compression_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->compression_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // target_framerate
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_framerate");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->target_framerate = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // input_transcode
    PyObject * field = PyObject_GetAttrString(_pymsg, "input_transcode");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->input_transcode = (Py_True == field);
    Py_DECREF(field);
  }
  {  // svo_filename
    PyObject * field = PyObject_GetAttrString(_pymsg, "svo_filename");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->svo_filename, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * zed_interfaces__srv__start_svo_rec__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StartSvoRec_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("zed_interfaces.srv._start_svo_rec");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StartSvoRec_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  zed_interfaces__srv__StartSvoRec_Request * ros_message = (zed_interfaces__srv__StartSvoRec_Request *)raw_ros_message;
  {  // bitrate
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->bitrate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bitrate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // compression_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->compression_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "compression_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_framerate
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->target_framerate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_framerate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // input_transcode
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->input_transcode ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "input_transcode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // svo_filename
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->svo_filename.data,
      strlen(ros_message->svo_filename.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "svo_filename", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "zed_interfaces/srv/detail/start_svo_rec__struct.h"
// already included above
// #include "zed_interfaces/srv/detail/start_svo_rec__functions.h"

// already included above
// #include "rosidl_runtime_c/string.h"
// already included above
// #include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool zed_interfaces__srv__start_svo_rec__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[55];
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
    assert(strncmp("zed_interfaces.srv._start_svo_rec.StartSvoRec_Response", full_classname_dest, 54) == 0);
  }
  zed_interfaces__srv__StartSvoRec_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * zed_interfaces__srv__start_svo_rec__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StartSvoRec_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("zed_interfaces.srv._start_svo_rec");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StartSvoRec_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  zed_interfaces__srv__StartSvoRec_Response * ros_message = (zed_interfaces__srv__StartSvoRec_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
