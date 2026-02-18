python3 -m venv k1_deploy
source k1_deploy/bin/activate

pip install pybind11 pybind11-stubgen

cd booster_robotics_sdk
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDING=on -DCMAKE_INSTALL_PREFIX=$(python3 -c "import site; print(site.getsitepackages()[0])")
make
make install
cd ../..

# booster_interface: source pre-built package on the robot,
# or build with colcon for dev environments.
if [ -f /opt/booster/BoosterRos2Interface/install/setup.bash ]; then
  source /opt/booster/BoosterRos2Interface/install/setup.bash
else
  echo "Pre-built booster_interface not found at /opt/booster/BoosterRos2Interface/install/setup.bash"
  echo "Building with colcon (requires ROS 2 environment sourced)..."
  mkdir -p /tmp/booster_ws/src
  ln -sf "$(pwd)/booster_ros2_interface" /tmp/booster_ws/src/booster_interface
  cd /tmp/booster_ws
  colcon build --packages-select booster_interface
  source install/setup.bash
  cd -
fi

pip install -r requirements.txt
