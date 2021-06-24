
cc_library(
    name = "tracker_helper",
    hdrs = ["tracker_helper.h"],
    srcs = ["tracker_helper.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "@opencv//:opencv",
    ]
)

cc_test(
    name = "tracker_helper_test",
    srcs = ["tracker_helper_test.cc"],
    linkstatic=1,
    deps = [
        ":tracker_helper",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)
