load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "commonfunctions",
    srcs = ["commonfunctions.cc"],
    hdrs = [
        "commonfunctions.h",
        ],
)

cc_library(
    name = "filter",
    srcs = ["filter.cc"],
    hdrs = [
        "filter.h",
        "dataStruct.h",
        "ContiRadar.h",
        "commonfunctions.h",
        ],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "match",
    srcs = ["match.cc"],
    hdrs = [
        "match.h",
        "dataStruct.h",
        "ContiRadar.h",
        "commonfunctions.h",
        ],
    deps = [
        "@eigen",
        "//modules/perception/radar/contiRadar/tracker/commonlib:DBSCAN",
        "//modules/perception/radar/contiRadar/tracker:commonfunctions"
    ],
)

cc_library(
    name = "motionStateDist",
    srcs = ["motionStateDist.cc"],
    hdrs = [
        "motionStateDist.h",
        "ContiRadar.h",
        "radar_install_message.h",
        "dataStruct.h",
        ],
    deps = [
        "@eigen",
    ],
)


cc_library(
    name = "preprocess",
    srcs = ["preprocess.cc"],
    hdrs = [
        "preprocess.h",
        "dataStruct.h",
        "ContiRadar.h",
        "motionStateDist.h",
        ],
    deps = [
        "@eigen",
        ":motionStateDist",
    ],
)


cc_library(
    name = "track_filter",
    srcs = ["track_filter.cc"],
    hdrs = [
        "track_filter.h",
        ],
    deps = [
        "@eigen",
        ":filter",
        ":match",
        "//modules/perception/radar/contiRadar/tracker/commonlib:DBSCAN",
        "//modules/perception/radar/contiRadar/tracker:commonfunctions"
    ],
)

cc_library(
    name = "track_manage",
    srcs = ["track_manage.cc"],
    hdrs = [
        "track_manage.h",
        ],
    deps = [
        "@eigen",
        ":filter",
        "//modules/perception/radar/contiRadar/tracker/commonlib:DBSCAN",
        "//modules/perception/radar/contiRadar/tracker:commonfunctions"
    ],
)

cc_library(
    name = "track_static_trace",
    srcs = ["track_static_trace.cc"],
    hdrs = [
        "track_static_trace.h",
        "dataStruct.h",
        "ContiRadar.h",
        ],
    deps = [
        "@eigen",
        ":filter",
        ":track_manage",
        "//modules/perception/radar/contiRadar/tracker/commonlib:DBSCAN",
        "//modules/perception/radar/contiRadar/tracker:commonfunctions"
    ],
)

cc_library(
    name = "track_process",
    srcs = ["track_process.cc"],
    hdrs = [
        "track_process.h",
        ],
    deps = [
        ":filter",
        ":match",
        ":preprocess",
        ":track_filter",
        ":track_manage",
        ":track_static_trace",
        "//modules/perception/radar/contiRadar/tracker/commonlib:DBSCAN",
        "@eigen"
    ],
)


cpplint()