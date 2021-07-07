load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

git_repository(
    name = "bazel_skylib",
    commit = "2ec2e6d",  # release 1.0.3
    remote = "https://github.com/bazelbuild/bazel-skylib.git",
)

http_archive(
    name = "zlib",
    build_file = "@com_google_protobuf//:third_party/zlib.BUILD",
    sha256 = "c3e5e9fdd5004dcb542feda5ee4f0ff0744628baf8ed2dd5d66f8ca1197cb1a1",
    strip_prefix = "zlib-1.2.11",
    urls = [
        "https://mirror.bazel.build/zlib.net/zlib-1.2.11.tar.gz",
        "https://zlib.net/zlib-1.2.11.tar.gz",
    ],
)

git_repository(
    name = "com_google_absl",
    commit = "6f9d96a", # release 20200923.3
    remote = "https://github.com/abseil/abseil-cpp.git",
)

git_repository(
    name = "rules_python",
    commit = "c8c79aa", #0.1.0
    remote = "https://github.com/bazelbuild/rules_python.git",
)

git_repository(
    name = "com_google_protobuf",
    commit = "983d115",  # release v3.15.3
    remote = "https://github.com/protocolbuffers/protobuf.git",
)
#load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

# glog
http_archive(
    name = "com_github_glog_glog",
    sha256 = "58c9b3b6aaa4dd8b836c0fd8f65d0f941441fb95e27212c5eeb9979cfd3592ab",
    strip_prefix = "glog-0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6",
    urls = [
        "https://github.com/google/glog/archive/0a2e5931bd5ff22fd3bf8999eb8ce776f159cda6.zip",
    ],
)

# gflags needed by glog
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "19713a36c9f32b33df59d1c79b4958434cb005b5b47dc5400a7a4b078111d9b5",
    strip_prefix = "gflags-2.2.2",
    url = "https://github.com/gflags/gflags/archive/v2.2.2.zip",
)

git_repository(
    name = "com_google_ortools",
    commit = "8d19323", # release v8.0
    remote = "https://github.com/google/or-tools.git",
)

http_archive(
    name = "bliss",
    build_file = "@com_google_ortools//bazel:bliss.BUILD",
    patches = ["@com_google_ortools//bazel:bliss-0.73.patch"],
    sha256 = "f57bf32804140cad58b1240b804e0dbd68f7e6bf67eba8e0c0fa3a62fd7f0f84",
    url = "http://www.tcs.hut.fi/Software/bliss/bliss-0.73.zip",
)

http_archive(
    name = "scip",
    build_file = "@com_google_ortools//bazel:scip.BUILD",
    patches = ["@com_google_ortools//bazel:scip.patch"],
    sha256 = "033bf240298d3a1c92e8ddb7b452190e0af15df2dad7d24d0572f10ae8eec5aa",
    url = "https://github.com/google/or-tools/releases/download/v7.7/scip-7.0.1.tgz",
)


# GoogleTest/GoogleMock framework. Used by most unit-tests.
# Last updated 2020-06-30.
http_archive(
    name = "com_google_googletest",
    sha256 = "04a1751f94244307cebe695a69cc945f9387a80b0ef1af21394a490697c5c895",
    strip_prefix = "googletest-aee0f9d9b5b87796ee8a0ab26b7587ec30e8858e",
    urls = ["https://github.com/google/googletest/archive/aee0f9d9b5b87796ee8a0ab26b7587ec30e8858e.zip"],
)

new_local_repository(
    name = "linux_opencv",
    build_file = "//third_party:opencv_linux.BUILD",
    path = "/usr/local",
)