cmake_minimum_required(VERSION 2.8.3)
project(segbot_description)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS
        base_sim_controller
        convex_decomposition
        ivcon
        pr2_controller_manager)

catkin_package()

# iterate through all the stl files to:
#  1. convert to *.iv files
#  2. generate convex decomposed .stl
file(GLOB segbot_description_stl_files ${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.stl)
set(segbot_description_gen_files "")

set(IV_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/meshes/iv)
set(CONVEX_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/meshes/convex)

foreach(it ${segbot_description_stl_files})
    get_filename_component(basepath ${it} PATH)
    get_filename_component(basename ${it} NAME_WE)

    if (${basename} MATCHES "_convex")
        message("ignoring stale .._convex.stl file: ${basename}")
    else (${basename} MATCHES "_convex")
        # create subdirectory convex
        add_custom_command(
                OUTPUT ${CONVEX_BINARY_DIR}
                COMMAND mkdir
                ARGS -p ${CONVEX_BINARY_DIR})

        # create subdirectory iv
        add_custom_command(
                OUTPUT ${IV_BINARY_DIR}
                COMMAND mkdir
                ARGS -p ${IV_BINARY_DIR})

        # conversion: *.stl -> convex/*.obj
        add_custom_command(
                OUTPUT ${CONVEX_BINARY_DIR}/${basename}.obj
                COMMAND ivcon
                ARGS ${it} ${CONVEX_BINARY_DIR}/${basename}.obj
                DEPENDS ${it} ${CONVEX_BINARY_DIR})

        set(segbot_description_gen_files ${segbot_description_gen_files} ${CONVEX_BINARY_DIR}/${basename}.obj)

        # convex_decomposition: convex/*.obj -> convex/*_convex.obj
        add_custom_command(
                OUTPUT ${CONVEX_BINARY_DIR}/${basename}_convex.obj
                COMMAND convex_decomposition
                ARGS ${CONVEX_BINARY_DIR}/${basename}.obj -v20 -p10
                DEPENDS ${CONVEX_BINARY_DIR}/${basename}.obj ${CONVEX_BINARY_DIR})

        set(segbot_description_gen_files ${segbot_description_gen_files} ${CONVEX_BINARY_DIR}/${basename}_convex.obj)

        # conversion: convex/*_convex.obj -> convex/*._convex.stlb
        add_custom_command(
            OUTPUT ${CONVEX_BINARY_DIR}/${basename}_convex.stlb
            COMMAND ivcon
            ARGS ${CONVEX_BINARY_DIR}/${basename}_convex.obj ${CONVEX_BINARY_DIR}/${basename}_convex.stlb
            DEPENDS ${it} ${CONVEX_BINARY_DIR} ${CONVEX_BINARY_DIR}/${basename}_convex.obj)

        set(segbot_description_gen_files ${segbot_description_gen_files} ${CONVEX_BINARY_DIR}/${basename}_convex.stlb)

        # convesion: convex/*_convex.obj -> convex/*_convex.stla
        add_custom_command(
                OUTPUT ${CONVEX_BINARY_DIR}/${basename}_convex.stla
                COMMAND ivcon
                ARGS ${CONVEX_BINARY_DIR}/${basename}_convex.obj ${CONVEX_BINARY_DIR}/${basename}_convex.stla
                DEPENDS ${it} ${CONVEX_BINARY_DIR} ${CONVEX_BINARY_DIR}/${basename}_convex.obj)

        set(segbot_description_gen_files ${segbot_description_gen_files} ${CONVEX_BINARY_DIR}/${basename}_convex.stla)

        # conversion: *.stl -> iv/*.iv
        add_custom_command(
                OUTPUT ${IV_BINARY_DIR}/${basename}.iv
                COMMAND ivcon
                ARGS ${it} ${IV_BINARY_DIR}/${basename}.iv
                DEPENDS ${IV_BINARY_DIR} ${it})

        # conversion convex/*_convex.obj -> convex/*_convex.iv
        add_custom_command(
                OUTPUT ${CONVEX_BINARY_DIR}/${basename}_convex.iv
                COMMAND ivcon
                ARGS ${CONVEX_BINARY_DIR}/${basename}_convex.obj ${CONVEX_BINARY_DIR}/${basename}_convex.iv
                DEPENDS ${it} ${CONVEX_BINARY_DIR} ${CONVEX_BINARY_DIR}/${basename}_convex.obj)

        set(segbot_description_gen_files ${segbot_description_gen_files} ${IV_BINARY_DIR}/${basename}.iv ${CONVEX_BINARY_DIR}/${basename}_convex.iv)
    endif (${basename} MATCHES "_convex")
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${segbot_description_gen_files})

install(DIRECTORY gazebo DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY robots DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
