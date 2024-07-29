load(
    "//bfc:copts/configure_copts.bzl",
    "BFC_DEFAULT_LINKOPTS",
)

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "atomic_hash_array",
    hdrs = [
        "AtomicHashArray.h",
        "AtomicHashArray-inl.h",
        "atomic_hash_array.h",
    ],
)

cc_test(
    name = "atomic_hash_array_test",
    size = "small",
    srcs = ["atomic_hash_array_test.cc"],
    linkopts = BFC_DEFAULT_LINKOPTS,
    linkstatic = True,
    deps = [
        ":atomic_hash_array",
        "@com_google_googletest//:gtest_main",
    ],
)
