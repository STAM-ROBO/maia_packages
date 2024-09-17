cmake_minimum_required(VERSION 3.0.2)
project(tcp_link_hybrid)

find_package(catkin REQUIRED)

catkin_package(
)

catkin_install_python(PROGRAMS scripts/tcp_link_hybrid.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
