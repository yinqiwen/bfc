load(
    "//bfc:copts/configure_copts.bzl",
    "BFC_DEFAULT_LINKOPTS",
)

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "mem_cache_bench",
    srcs = ["mem_cache_bench.cc"],
    linkopts = BFC_DEFAULT_LINKOPTS,
    linkstatic = True,
    deps = [
        "//bfc/cache:mem_cache",
    ],
)

cc_binary(
    name = "disk_cache_bench",
    srcs = ["disk_cache_bench.cc"],
    linkopts = BFC_DEFAULT_LINKOPTS,
    linkstatic = True,
    deps = [
        "//bfc/cache:disk_cache",
    ],
)
