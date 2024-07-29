load(
    "//bfc:copts/configure_copts.bzl",
    "BFC_DEFAULT_LINKOPTS",
)

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "timer",
    srcs = [
        "timer.cc",
    ],
    hdrs = [
        "timer.h",
    ],
)

cc_test(
    name = "timer_test",
    size = "small",
    srcs = ["timer_test.cc"],
    linkopts = BFC_DEFAULT_LINKOPTS,
    linkstatic = True,
    deps = [
        ":timer",
        "@com_google_googletest//:gtest_main",
    ],
)
