This package should not be used unless the Trajopt package is already installed. 
Instructions for installing Trajopt can be found here: http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/install.html

Troubleshooting:
- The troubleshooting tips on the installation page are good. In particular, make sure your PYTHONPATH and LD_LIBRARY_PATH are set according to the instructions
- If you get an error related to bullet, then do 'sudo apt-get remove openrave0.9-dp-plugin-bulletrave' for the right version of openrave
- We found that it's best not to have any ROS distro sourced while building Trajopt. Once you build it, you can source your ROS distro when you're using Trajopt
- If you get Assembler errors when building then add this flag to your CMakeLists.txt: set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mno-avx")


