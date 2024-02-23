# pcl_interface

## Dependencies

curvature_computation originally from https://github.com/HeCraneChen/Open3D-curvature-computation

## To build curvature_computation

```
# Update cmake version > 3.18
# https://cmake.org/download/
sudo ln -s /full/path/to/cmake/bin/* /usr/local/bin
cmake --version

# Build open3d from source
https://www.open3d.org/docs/release/compilation.html


# Build curvature_computation
pip3 install "pybind11[global]"
cd 3rdparty/curvature_computation
mkdir build
cd build
cmake ..
make
sudo make install
python -c "import curvature_computation"
```
