load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools/proto:proto.bzl", "proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "speed_bump_speed_limit_proto",
    srcs = ["speed_bump_speed_limit.proto"],
)

apollo_package()

cpplint()