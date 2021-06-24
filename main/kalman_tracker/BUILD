
cc_library(
    name = "kalman_tracker",
    hdrs = ["kalman_tracker.h"],
    srcs = ["kalman_tracker.cc"],
    deps = [
        "//main/utils:tracker_helper",
        "@opencv//:opencv",
        "@com_google_ortools//ortools/base",
        "@com_google_ortools//ortools/graph",
        "@com_google_ortools//ortools/graph:linear_assignment",
        "@com_google_ortools//ortools/algorithms:hungarian",
        "@com_google_absl//absl/container:flat_hash_map"
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "kalman_tracker_test",
    srcs = ["kalman_tracker_test.cc"],
    linkstatic=1,
    deps = [
        ":kalman_tracker",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)
