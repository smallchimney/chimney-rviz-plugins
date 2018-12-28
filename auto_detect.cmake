macro(CHIMNEY_INCLUDE_ALL include_root)
    # find all dir in include/
    FILE(GLOB dirs
            RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
            ${include_root}/*
    )
    FILE(GLOB ROOT_FILES
            RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
            ${include_root}/*.h
    )
    if(ROOT_FILES)
        LIST(REMOVE_ITEM dirs ${ROOT_FILES})
        LIST(APPEND HEADER_FILES ${ROOT_FILES})
    endif()
    # remove unsolved file
    if(ignored_include_dirs)
        LIST(REMOVE_ITEM dirs ${ignored_header_dirs})
    endif()
    FOREACH(target ${dirs})
        MESSAGE(STATUS "loading the dir: ${target}")
        # search for all h files in target dir
        FILE(GLOB_RECURSE ${target}_include
                RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
                *.h
        )
        LIST(APPEND HEADER_FILES ${${target}_include})
    ENDFOREACH()
endmacro()

macro(CHIMNEY_SOURCE_ALL source_root)
    FILE(GLOB dirs
            RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
            ${source_root}/*
    )
    FILE(GLOB ROOT_FILES
            RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
            ${source_root}/*.c
            ${source_root}/*.cpp
    )
    if(ROOT_FILES)
        LIST(REMOVE_ITEM dirs ${ROOT_FILES})
        LIST(APPEND SRC_FILES ${ROOT_FILES})
    endif()
    # remove unsolved file
    if(ignored_src_dirs)
        LIST(REMOVE_ITEM dirs ${ignored_src_dirs})
    endif()
    FOREACH(target ${dirs})
        MESSAGE(STATUS "loading the dir: ${target}")
        # search for all cpp files in target dir
        FILE(GLOB_RECURSE ${target}_srcs
                RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
                *.c
                *.cpp
                )
        LIST(APPEND SRC_FILES ${${target}_srcs})
    ENDFOREACH()
endmacro()