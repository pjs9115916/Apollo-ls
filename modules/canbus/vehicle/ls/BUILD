load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ls_vehicle_factory",
    srcs = [
        "ls_vehicle_factory.cc",
    ],
    hdrs = [
        "ls_vehicle_factory.h",
    ],
    deps = [
        ":ls_controller",
        ":ls_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "ls_message_manager",
    srcs = [
        "ls_message_manager.cc",
    ],
    hdrs = [
        "ls_message_manager.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "ls_controller",
    srcs = [
        "ls_controller.cc",
    ],
    hdrs = [
        "ls_controller.h",
    ],
    deps = [
        ":ls_message_manager",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ls_controller_test",
    size = "small",
    srcs = [
        "ls_controller_test.cc",
    ],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":ls_controller",
        "@gtest//:main",
    ],
)

cc_test(
    name = "ls_message_manager_test",
    size = "small",
    srcs = [
        "ls_message_manager_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls:ls_message_manager",
        "@gtest//:main",
    ],
)

cc_test(
    name = "ls_vehicle_factory_test",
    size = "small",
    srcs = ["ls_vehicle_factory_test.cc"],
    deps = [
        ":ls_vehicle_factory",
        "@gtest//:main",
    ],
)

cpplint()
