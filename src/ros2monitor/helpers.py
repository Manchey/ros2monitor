#!/usr/bin/env python

import importlib

from ament_index_python import get_resource

from rclpy import logging

MSG_MODE = "msg"
SRV_MODE = "srv"
ACTION_MODE = "action"

ROSIDL_FILTERS = {
    MSG_MODE: lambda n: n.startswith("msg/") and n.endswith(".msg"),
    SRV_MODE: lambda n: n.startswith("srv/") and n.endswith(".srv"),
    ACTION_MODE: lambda n: n.startswith("action/") and n.endswith(".idl"),
}


def _get_rosidl_class_helper(message_type, mode, logger=None):  # noqa: C901
    """
    Help to get_message_class and get_service_class.

    :param message_type: name of the message or service class in the form
      'package_name/MessageName' or 'package_name/msg/MessageName'
    :type message_type: str
    :param mode: one of MSG_MODE, SRV_MODE or ACTION_MODE
    :type mode: str
    :param logger: The logger to be used for warnings and info
    :type logger: either rclpy.impl.rcutils_logger.RcutilsLogger or None

    :returns: The message or service class or None
    """
    if logger is None:
        logger = logging.get_logger("_get_message_service_class_helper")

    if mode not in ROSIDL_FILTERS.keys():
        logger.warn("invalid mode {}".format(mode))
        return None

    message_info = message_type.split("/")
    if len(message_info) not in (2, 3):
        logger.error("Malformed message_type: {}".format(message_type))
        return None
    if len(message_info) == 3 and message_info[1] != mode:
        logger.error("Malformed {} message_type: {}".format(mode, message_type))
        return None

    package = message_info[0]
    base_type = message_info[-1]

    try:
        _, resource_path = get_resource("rosidl_interfaces", package)
    except LookupError:
        return None
    python_pkg = None
    class_val = None

    try:
        # import the package
        python_pkg = importlib.import_module("%s.%s" % (package, mode))
    except ImportError:
        logger.info("Failed to import class: {} as {}.{}".format(message_type, package, mode))
        return None

    try:
        class_val = getattr(python_pkg, base_type)
        return class_val

    except AttributeError:
        logger.info("Failed to load class: {}".format(message_type))
        return None


_message_class_cache = {}


def get_message_class(message_type):
    """
    Get the message class from a string representation.

    :param message_type: the type of message in the form `msg_pkg/Message`
    :type message_type: str

    :returns: None or the Class
    """
    if message_type in _message_class_cache:
        return _message_class_cache[message_type]

    logger = logging.get_logger("get_message_class")

    class_val = _get_rosidl_class_helper(message_type, MSG_MODE, logger)

    if class_val is not None:
        _message_class_cache[message_type] = class_val

    return class_val
