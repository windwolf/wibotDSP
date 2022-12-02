cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

set(CMSISDSP CMSISDSP)

process_src_dir(${CMAKE_CURRENT_LIST_DIR}/basic ${PROJECT_NAME})
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/CMSIS-DSP/Source)

target_link_libraries(${PROJECT_NAME} PRIVATE CMSISDSP)
