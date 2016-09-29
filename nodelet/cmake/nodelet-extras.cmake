######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2016, Kentaro Wada.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Kentaro Wada nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

macro(nodelet_add_executable node_name nodelet_name)
  # Get path to nodelet ROS package
  if(nodelet_SOURCE_DIR)
    set(_nodelet_SOURCE_DIR ${nodelet_SOURCE_DIR})
  elseif(nodelet_SOURCE_PREFIX)
    set(_nodelet_SOURCE_DIR ${nodelet_SOURCE_PREFIX})
  else(nodelet_SOURCE_PREFIX)
    set(_nodelet_SOURCE_DIR ${nodelet_PREFIX}/share/nodelet)
  endif()
  # Generate single nodelet executable
  set(NODE_NAME ${node_name})
  set(NODELET_NAME ${nodelet_name})
  configure_file(${_nodelet_SOURCE_DIR}/cmake/standalone_nodelet_exe.cpp.in
                 ${node_name}.cpp @ONLY)
  add_executable(${node_name}_exe ${node_name}.cpp)
  set_target_properties(${node_name}_exe PROPERTIES OUTPUT_NAME ${node_name})
  # Additional linking should be conducted in each package
  target_link_libraries(${node_name}_exe ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
  # Generate list of targets to use at installation
  list(APPEND ${PROJECT_NAME}_nodelet_exe_targets ${node_name}_exe)
endmacro()