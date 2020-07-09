load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_ls_protocol",
    deps = [
        ":brake_command_102",
        ":brake_status__202",
        ":control_command_100",
        ":ecu_status__200",
        ":gear_command_104",
        ":gear_status__204",
        ":steer_command_103",
        ":steer_status__203",
        ":throttle_command_101",
        ":throttle_status__201",
        ":turnsignal_command_104",
        ":turnsignal_status__204",
    ],
)

cc_library(
    name = "brake_command_102",
    srcs = [
        "brake_command_102.cc",
    ],
    hdrs = [
        "brake_command_102.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "brake_command_102_test",
    size = "small",
    srcs = [
        "brake_command_102_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cc_library(
    name = "brake_status__202",
    srcs = [
        "brake_status__202.cc",
    ],
    hdrs = [
        "brake_status__202.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "brake_status__202_test",
    size = "small",
    srcs = [
        "brake_status__202_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cc_library(
    name = "control_command_100",
    srcs = [
        "control_command_100.cc",
    ],
    hdrs = [
        "control_command_100.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "control_command_100_test",
    size = "small",
    srcs = [
        "control_command_100_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cc_library(
    name = "ecu_status__200",
    srcs = [
        "ecu_status__200.cc",
    ],
    hdrs = [
        "ecu_status__200.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ecu_status__200_test",
    size = "small",
    srcs = [
        "ecu_status__200_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)



cc_library(
    name = "gear_command_104",
    srcs = [
        "gear_command_104.cc",
    ],
    hdrs = [
        "gear_command_104.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gear_command_104_test",
    size = "small",
    srcs = [
        "gear_command_104_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cc_library(
    name = "gear_status__204",
    srcs = [
        "gear_status__204.cc",
    ],
    hdrs = [
        "gear_status__204.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gear_status__204_test",
    size = "small",
    srcs = [
        "gear_status__204_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cc_library(
    name = "steer_command_103",
    srcs = [
        "steer_command_103.cc",
    ],
    hdrs = [
        "steer_command_103.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "steer_command_103_test",
    size = "small",
    srcs = [
        "steer_command_103_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cc_library(
    name = "steer_status__203",
    srcs = [
        "steer_status__203.cc",
    ],
    hdrs = [
        "steer_status__203.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "steer_status__203_test",
    size = "small",
    srcs = [
        "steer_status__203_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cc_library(
    name = "throttle_command_101",
    srcs = [
        "throttle_command_101.cc",
    ],
    hdrs = [
        "throttle_command_101.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "throttle_command_101_test",
    size = "small",
    srcs = [
        "throttle_command_101_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cc_library(
    name = "throttle_status__201",
    srcs = [
        "throttle_status__201.cc",
    ],
    hdrs = [
        "throttle_status__201.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "throttle_status__201_test",
    size = "small",
    srcs = [
        "throttle_status__201_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cc_library(
    name = "turnsignal_command_104",
    srcs = [
        "turnsignal_command_104.cc",
    ],
    hdrs = [
        "turnsignal_command_104.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "turnsignal_command_104_test",
    size = "small",
    srcs = [
        "turnsignal_command_104_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cc_library(
    name = "turnsignal_status__204",
    srcs = [
        "turnsignal_status__204.cc",
    ],
    hdrs = [
        "turnsignal_status__204.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "turnsignal_status__204_test",
    size = "small",
    srcs = [
        "turnsignal_status__204_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ls/protocol:canbus_ls_protocol",
        "@gtest//:main",
    ],
)

cpplint()
