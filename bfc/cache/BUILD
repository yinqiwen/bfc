load(
    "//bfc:copts/configure_copts.bzl",
    "BFC_DEFAULT_LINKOPTS",
)

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "options",
    hdrs = [
        "options.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "context",
    hdrs = [
        "context.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "address",
    hdrs = [
        "address.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "hash_bucket",
    hdrs = [
        "hash_bucket.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "kv_bucket",
    hdrs = [
        "kv_bucket.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "types",
    hdrs = [
        "types.h",
    ],
    deps = [
        ":address",
        "//bfc/common:time_helper",
    ],
)

cc_library(
    name = "stats",
    hdrs = [
        "stats.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "mem_cache",
    hdrs = [
        "mem_cache.h",
    ],
    deps = [
        ":kv_bucket",
        ":stats",
        ":types",
        "//bfc/cache:options",
        "//bfc/log",
        "//bfc/timer",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "disk_cache",
    hdrs = [
        "disk_cache.h",
    ],
    deps = [
        ":hash_bucket",
        ":options",
        ":stats",
        ":types",
        "//bfc/common:rate_limit_file",
        "//bfc/common:segment_store",
        "//bfc/log",
        "//bfc/timer",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_test(
    name = "kv_bucket_test",
    size = "small",
    srcs = ["kv_bucket_test.cc"],
    linkopts = BFC_DEFAULT_LINKOPTS,
    linkstatic = True,
    deps = [
        ":kv_bucket",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "mem_cache_test",
    size = "small",
    srcs = ["mem_cache_test.cc"],
    linkopts = BFC_DEFAULT_LINKOPTS,
    linkstatic = True,
    deps = [
        ":mem_cache",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "disk_cache_test",
    size = "small",
    srcs = ["disk_cache_test.cc"],
    linkopts = BFC_DEFAULT_LINKOPTS,
    linkstatic = True,
    deps = [
        ":disk_cache",
        "@com_google_googletest//:gtest_main",
    ],
)
