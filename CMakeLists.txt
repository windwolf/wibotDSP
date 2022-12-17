cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

set(CMSISDSP CMSISDSP)


add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/CMSIS-DSP/Source)
process_src_dir(${CMAKE_CURRENT_LIST_DIR}/basic ${PROJECT_NAME})
process_src_dir(${CMAKE_CURRENT_LIST_DIR}/filter ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} PRIVATE CMSISDSP)
