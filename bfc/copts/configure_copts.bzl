BFC_DEFAULT_COPTS = [
    "-D__STDC_FORMAT_MACROS",
    "-D__STDC_LIMIT_MACROS",
    "-D__STDC_CONSTANT_MACROS",
    "-DGFLAGS_NS=google",
    "-Werror=return-type",
]

BFC_DEFAULT_LINKOPTS = [
    "-L/usr/local/lib",
    "-L/usr/local/lib64",
    "-L/usr/local/cuda/lib64",
    "-lfolly",
    "-lglog",
    "-lgflags",
    "-lfmt",
    "-ldouble-conversion",
    "-liberty",
    "-levent",
    "-lunwind",
    "-lcrypto",
    "-lssl",
    "-ldl",
    "-lrt",
    "-lstdc++fs",
    "-lboost_context",
    "-lboost_filesystem",
    "-lboost_program_options",
]
