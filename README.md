
# Introduction
SORT Tracker with OpenCV KalmanFilter and OR-Tools linear assignment solver.

<a name="deps"></a>
# Dependencies
If you don't have Bazel installed already, you can download it for free from
<https://bazel.build/>.

Take a look at the [WORKSPACE](WORKSPACE) file.

<a name="build"></a>
# Build
On any *\*NIX* (MacOS, GNU/Linux) platform:
```sh
bazel build --cxxopt=-std=c++17 //main:template
```

Run Unit Tests
```sh
bazel test //main/kalman_tracker:kalman_tracker_test --test_output=all
```