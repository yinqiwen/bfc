package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cksm",
    srcs = ["cksm.cc"],
    hdrs = [
        "cksm.h",
    ],
    deps = [
        "@com_github_cyan4973_xxhash//:xxhash",
    ],
)

cc_library(
    name = "time_helper",
    srcs = ["time_helper.cc"],
    hdrs = [
        "time_helper.h",
    ],
)

cc_library(
    name = "cached_time",
    srcs = ["cached_time.cc"],
    hdrs = [
        "cached_time.h",
    ],
    deps = [
        ":time_helper",
        "//bfc/timer",
    ],
)

cc_library(
    name = "rate_limit_file",
    srcs = ["rate_limit_file.cc"],
    hdrs = [
        "rate_limit_file.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "mmap_helper",
    srcs = [
        "mmap_helper.cc",
    ],
    hdrs = [
        "mmap_helper.h",
    ],
    deps = [
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "segment_store",
    srcs = [
        "segment_store.cc",
    ],
    hdrs = [
        "segment_store.h",
    ],
    deps = [
        ":mmap_helper",
        ":time_helper",
        "//bfc/log",
        "//bfc/timer",
        "@com_google_absl//absl/status:statusor",
    ],
)
