load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "DBSCAN",
    srcs = ["DBSCAN.cc"],
    hdrs = [
        "DBSCAN.h",
        ],
    deps = [
        "@eigen",
    ],
)

cpplint()