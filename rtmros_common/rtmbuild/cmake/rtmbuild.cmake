cmake_minimum_required(VERSION 2.4.6)

set(_RTMBUILD_GENERATED_IDL_FILES "")
macro(rtmbuild_get_idls idlvar)
  file(GLOB _idl_files RELATIVE "${PROJECT_SOURCE_DIR}/idl" "${PROJECT_SOURCE_DIR}/idl/*.idl")
  set(${idlvar} ${_RTMBUILD_GENERATED_IDL_FILES})
  foreach(_idl ${_idl_files})
    # copy from rosbuild_get_msgs to avoid .#Foo.idl, by emacs
    if(${_idl} MATCHES "^[^\\.].*\\.idl$")
      list(APPEND ${idlvar} ${_idl})
    endif(${_idl} MATCHES "^[^\\.].*\\.idl$")
  endforeach(_idl)
endmacro(rtmbuild_get_idls)

#compile idl/*.idl file into c++
macro(rtmbuild_genidl)
  message("[rtmbuild_genidl] Compiling idls in package ${_project}")
  rtmbuild_get_idls(_idllist)
  set(_autogen "")
  if(NOT _idllist)
    _rosbuild_warn("rtmbuild_genidl() was called, but no .idl files ware found")
  else(NOT _idllist)
    file(WRITE ${PROJECT_SOURCE_DIR}/idl_gen/generated "yes")
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/idl_gen/cpp/${_project}/idl)
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/idl_gen/lib)
  endif(NOT _idllist)

  set(_output_dir ${PROJECT_SOURCE_DIR}/idl_gen)
  set(_output_cpp_dir ${PROJECT_SOURCE_DIR}/idl_gen/cpp/${_project}/idl)
  set(_output_python_dir ${PROJECT_SOURCE_DIR}/src/${_project})
  set(_output_lib_dir ${PROJECT_SOURCE_DIR}/idl_gen/lib)

  foreach(_idl ${_idllist})
    message("[rtmbuild_genidl] ${_idl}")
    execute_process(COMMAND ${_openrtm_pkg_dir}/bin/rtm-config --idlc OUTPUT_VARIABLE _genidl_exe
      OUTPUT_STRIP_TRAILING_WHITESPACE)
#    --cxx is grx option???
#    execute_process(COMMAND rtm-config --cxx OUTPUT_VARIABLE _rtmcxx_ex
#      OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(_rtmcxx_exe "g++")

    set(_input_idl ${PROJECT_SOURCE_DIR}/idl/${_idl})
    set(_output_idl_hh ${_output_cpp_dir}/${_idl})
    set(_output_idl_py ${PROJECT_SOURCE_DIR}/src/${_project}/${_idl})
    set(_output_stub_cpp ${_output_cpp_dir}/${_idl})
    set(_output_skel_cpp ${_output_cpp_dir}/${_idl})
    set(_output_stub_lib ${_output_lib_dir}/lib${_idl})
    set(_output_skel_lib ${_output_lib_dir}/lib${_idl})
    string(REPLACE ".idl" ".hh" _output_idl_hh ${_output_idl_hh})
    string(REPLACE ".idl" "_idl.py" _output_idl_py ${_output_idl_py})
    string(REPLACE ".idl" "Stub.cpp" _output_stub_cpp ${_output_stub_cpp})
    string(REPLACE ".idl" "Stub.so"  _output_stub_lib ${_output_stub_lib})
    string(REPLACE ".idl" "Skel.cpp" _output_skel_cpp ${_output_skel_cpp})
    string(REPLACE ".idl" "Skel.so"  _output_skel_lib ${_output_skel_lib})

    # call the  rule to compile idl
    add_custom_command(OUTPUT ${_output_idl_hh}
      COMMAND ${_genidl_exe} `${_openrtm_pkg_dir}/bin/rtm-config --idlflags` `${_openrtm_pkg_dir}/bin/rtm-config --cflags | sed 's/^-[^I]\\S*//g' | sed 's/\ -[^I]\\S*//g'` -I${_openhrp3_pkg_dir}/share/OpenHRP-3.1/idl -C${_output_cpp_dir} ${_input_idl}
      DEPENDS ${_input_idl} ${${_idl}_depends})
    add_custom_command(OUTPUT ${_output_stub_cpp} ${_output_skel_cpp}
      COMMAND cp ${_input_idl} ${_output_cpp_dir}
      COMMAND ${_openrtm_pkg_dir}/bin/rtm-skelwrapper --include-dir="" --skel-suffix=Skel --stub-suffix=Stub  --idl-file=${_idl}
      COMMAND rm ${_output_cpp_dir}/${_idl}
      WORKING_DIRECTORY ${_output_cpp_dir}
      DEPENDS ${_output_idl_hh})
    add_custom_command(OUTPUT ${_output_stub_lib}
      COMMAND ${_rtmcxx_exe} `${_openrtm_pkg_dir}/bin/rtm-config --cflags` -I. ${${_prefix}_IDLLIBRARY_INCDIRS} -shared -o ${_output_stub_lib} ${_output_stub_cpp} `${_openrtm_pkg_dir}/bin/rtm-config --libs` ${OPENHRP_PRIVATE_LIBRARIES}
      DEPENDS ${_output_stub_cpp})
    add_custom_command(OUTPUT ${_output_skel_lib}
      COMMAND ${_rtmcxx_exe} `${_openrtm_pkg_dir}/bin/rtm-config --cflags` -I. ${${_prefix}_IDLLIBRARY_INCDIRS} -shared -o ${_output_skel_lib} ${_output_skel_cpp} `${_openrtm_pkg_dir}/bin/rtm-config --libs` ${OPENHRP_PRIVATE_LIBRARIES}
      DEPENDS ${_output_skel_cpp})
    # python
    add_custom_command(OUTPUT ${_output_idl_py}
      COMMAND mkdir -p ${_output_python_dir}
      COMMAND echo \"import sys\; sys.path.append('${PROJECT_SOURCE_DIR}/src/${_project}')\; import ${_project}\" > ${_output_python_dir}/__init__.py
      COMMAND ${_genidl_exe} -bpython -I`${_openrtm_pkg_dir}/bin/rtm-config --cflags | sed 's/^-[^I]\\S*//g' | sed 's/\ -[^I]\\S*//g'` -I${_openhrp3_pkg_dir}/share/OpenHRP-3.1/idl -C${_output_python_dir} ${_input_idl}
      DEPENDS ${_input_idl} ${${_idl}_depends})
    #
    list(APPEND _autogen ${_output_stub_lib} ${_output_skel_lib} ${_output_idl_py})
  endforeach(_idl)
  ##
  add_dependencies(rtmbuild_genidl RTMBUILD_genidl)
  add_custom_target(RTMBUILD_genidl ALL DEPENDS ${_autogen})

  #
  if(_autogen)
    # Also set up to clean the srv_gen directory
    get_directory_property(_old_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _old_clean_files ${PROJECT_SOURCE_DIR}/idl_gen)
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_old_clean_files}")
  endif(_autogen)
endmacro(rtmbuild_genidl)

##
macro(rtmbuild_init)
  rosbuild_init()
  message("[rtmbuild] Building package ${_project}")
  add_custom_target(rtmbuild_genidl ALL)
  file(REMOVE ${PROJECT_SOURCE_DIR}/idl_gen/generated)

  list(APPEND ${_prefix}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/idl_gen/cpp/${_prefix}/idl)
  list(APPEND ${_prefix}_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/idl_gen/lib)
  list(APPEND ${_prefix}_IDL_DIRS ${PROJECT_SOURCE_DIR}/idl)

  rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} DIRS depends-manifests)
  foreach(_dir ${${_prefix}_DIRS})
    string(REPLACE "manifest.xml" "idl_gen/cpp" _cpp_dir ${_dir})
    list(APPEND ${_prefix}_INCLUDE_DIRS ${_cpp_dir})
    string(REGEX REPLACE ".*/([^/]*)/manifest.xml" "\\1" _tmp_pkg ${_dir})
    list(APPEND ${_prefix}_INCLUDE_DIRS ${_cpp_dir}/${_tmp_pkg}/idl)
    string(REPLACE "manifest.xml" "idl_gen/lib" _lib_dir ${_dir})
    list(APPEND ${_prefix}_LIBRARY_DIRS ${_lib_dir})
    string(REPLACE "manifest.xml" "idl" _idl_dir ${_dir})
    list(APPEND ${_prefix}_IDL_DIRS ${_idl_dir})
  endforeach(_dir)

  ## use pkgconfig with openhrp3.1 and openrtm-aist
  # catch the error output to suppress it
  include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
  execute_process(COMMAND rospack find openrtm OUTPUT_VARIABLE _openrtm_pkg_dir OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND rospack find openhrp3 OUTPUT_VARIABLE _openhrp3_pkg_dir OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(ENV{PKG_CONFIG_PATH} "${_openhrp3_pkg_dir}/lib/pkgconfig:${_openrtm_pkg_dir}/lib/pkgconfig")
  #set(ENV{PKG_CONFIG_PATH} "${_openhrp3_pkg_dir}/lib/pkgconfig:/opt/grx/lib/pkgconfig")
  pkg_check_modules(OPENRTM REQUIRED openrtm-aist)
  pkg_check_modules(OPENHRP openhrp3.1)
  list(APPEND ${_prefix}_INCLUDE_DIRS ${OPENRTM_INCLUDE_DIRS})
  list(APPEND ${_prefix}_INCLUDE_DIRS ${OPENHRP_INCLUDE_DIRS})
  list(APPEND ${_prefix}_LIBRARY_DIRS ${OPENRTM_LIBRARY_DIRS})
  list(APPEND ${_prefix}_LIBRARY_DIRS ${OPENHRP_LIBRARY_DIRS})
  include_directories(${${_prefix}_INCLUDE_DIRS})
  link_directories(${${_prefix}_LIBRARY_DIRS})
  foreach(_lib ${OPENRTM_LIBRARIES})
    list(APPEND ${_prefix}_LIBRARIES ${_lib})
  endforeach(_lib)
  foreach(_lib ${OPENHRP_LIBRARIES})
    list(APPEND ${_prefix}_LIBRARIES ${_lib})
  endforeach(_lib)

  # get all libraries
  foreach(_idl_dir ${${_prefix}_IDL_DIRS})
    file(GLOB _idl_files "${_idl_dir}/*.idl")
    foreach(_idl_file ${_idl_files})
      string(REPLACE "/idl/" "/idl_gen/lib/lib" _idl_file ${_idl_file})
      string(REPLACE ".idl" "Skel.so" _idl_skel_file ${_idl_file})
      string(REPLACE ".idl" "Stub.so" _idl_stub_file ${_idl_file})
      list(APPEND ${_prefix}_IDLLIBRARY_DIRS ${_idl_skel_file})
      list(APPEND ${_prefix}_IDLLIBRARY_DIRS ${_idl_stub_file})
    endforeach(_idl_file)
    string(REGEX REPLACE ".*/([^/]*)/idl" "\\1" _tmp_pkg ${_idl_dir})
    list(APPEND ${_prefix}_IDLLIBRARY_IDLDIRS "-I${_idl_dir}")
    list(APPEND ${_prefix}_IDLLIBRARY_INCDIRS "-I${_idl_dir}/../idl_gen/cpp/${_tmp_pkg}/idl")
  endforeach(_idl_dir)
endmacro(rtmbuild_init)

macro(rtmbuild_add_executable exe)
  rosbuild_add_executable(${ARGV})
  add_dependencies(${exe} RTMBUILD_genidl)
  target_link_libraries(${exe} ${${_prefix}_IDLLIBRARY_DIRS})
#  rosbuild_add_executable(${ARGV})
#  message(${${PROJECT_NAME}_CFLAGS_OTHER})
#  rosbuild_add_compile_flags(${exe} ${${PROJECT_NAME}_CFLAGS_OTHER})
#  rosbuild_add_link_flags(${exe} ${${PROJECT_NAME}_LDFLAGS_OTHER})
#  target_link_libraries(${exe} ${PROJECT_NAME}_LD)
endmacro(rtmbuild_add_executable)

macro(rtmbuild_add_library lib)
  rosbuild_add_library(${ARGV})
  add_dependencies(${lib} RTMBUILD_genidl)
  target_link_libraries(${lib} ${${_prefix}_IDLLIBRARY_DIRS})
#  rosbuild_add_library(${ARGV})
#  message(${${PROJECT_NAME}_CFLAGS_OTHER})
#  rosbuild_add_compile_flags(${exe} ${${PROJECT_NAME}_CFLAGS_OTHER})
#  rosbuild_add_link_flags(${exe} ${${PROJECT_NAME}_LDFLAGS_OTHER})
#  target_link_libraries(${exe} ${PROJECT_NAME}_LD)
endmacro(rtmbuild_add_library)


# generate msg/srv files from idl, must be called before rosbuild_init
macro(rtmbuild_genbridge)
  message("[rtmbuild_genbridge] Generating bridge compornents from idl in package ${_project}")
  rtmbuild_get_idls(_idllist)
  if(NOT _idllist)
    _rosbuild_warn("rtmbuild_genbridge() was called, but no .idl files ware found")
  else(NOT _idllist)
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/msg)
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/srv)
  endif(NOT _idllist)

  execute_process(COMMAND rospack find openhrp3 OUTPUT_VARIABLE _openhrp3_pkg_dir OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND rosrun openrtm rtm-config --cflags | sed 's/ -[^I]\\S*//g' | sed 's/^-[^I]\\S*//g' | sed 's/-I//g' OUTPUT_VARIABLE _rtm_include_dir OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(_include_dirs "${_rtm_include_dir} ${_openhrp3_pkg_dir}/share/OpenHRP-3.1/idl")

  set(_autogen "")
  include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
  foreach(_idl ${_idllist})
    message("[rtmbuild_genbridge] Generate srvs from ${_idl}")
    execute_process(COMMAND rosrun rtmbuild idl2srv.py --filenames -i ${PROJECT_SOURCE_DIR}/idl/${_idl} --include-dirs="${_include_dirs}" OUTPUT_VARIABLE _autogen_files
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REPLACE "\n" ";" _autogen_files  ${_autogen_files})
    #add_custom_target(OUTPUT ${_autogen_files}
    #  COMMAND rosrun rtmbuild idl2srv.py -i ${PROJECT_SOURCE_DIR}/idl/${_idl} --include-dirs="${_include_dirs}"
    #  DEPENDS ${PROJECT_SOURCE_DIR}/idl/${_idl})
    message("rosrun rtmbuild idl2srv.py -i ${PROJECT_SOURCE_DIR}/idl/${_idl} --include-dirs=\"${_include_dirs}\"")
    execute_process(COMMAND rosrun rtmbuild idl2srv.py -i ${PROJECT_SOURCE_DIR}/idl/${_idl} --include-dirs="${_include_dirs}")
    list(APPEND _autogen ${_autogen_files})
  endforeach(_idl)

  if(_autogen)
    # Also set up to clean the generated msg/srv/cpp/h files
    get_directory_property(_old_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _old_clean_files ${_autogen})
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_old_clean_files}")
  endif(_autogen)

endmacro(rtmbuild_genbridge)

# rtm/ros bridge executables
macro(rtmros_bridge_executables)
  rosbuild_genmsg() # where to call ?
  rosbuild_gensrv() #
  rtmbuild_get_idls(_idllist)
  foreach(_idl ${_idllist})
    execute_process(COMMAND rosrun rtmbuild idl2srv.py --interfaces -i ${PROJECT_SOURCE_DIR}/idl/${_idl} --include-dirs="${_include_dirs}" OUTPUT_VARIABLE _interface
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REPLACE "\n" ";" _interface ${_interface})
    foreach(_comp ${_interface})
      message("[rtmbuild_bridge_executables] ${_idl} -> ${_comp}ROSBridgeComp")
      rtmbuild_add_executable("${_comp}ROSBridgeComp" "src/${_comp}ROSBridge.cpp" "src/${_comp}ROSBridgeComp.cpp")
    endforeach(_comp)
  endforeach(_idl)
endmacro(rtmros_bridge_executables)

