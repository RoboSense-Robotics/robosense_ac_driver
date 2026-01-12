#========================
# libs
#========================

# set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CUR_SRCS "")
set(CUR_CU_SRCS "")
set(CUR_INCLUDES "include")


set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
    foreach(f ${tmp_srcs}) 
        if((NOT f MATCHES ".*/gpujpeg*.*") AND (NOT f MATCHES ".*/nv12_rgb_yuv422.*") AND (NOT f MATCHES ".*/GPUJPEG*.*"))
                # message("===> f = ${f}") 
                list(APPEND CUR_SRCS ${f}) 
        endif() 
    endforeach() 
    # list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

if(CUDA_FOUND) 
    LIST(APPEND CUR_INCLUDES "include/hyper_vision/codec")
    LIST(APPEND CUR_INCLUDES "src/GPUJPEG/src")

    message("CUDA_FOUND: "${CUDA_FOUND})
    enable_language(CUDA)
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++14")
    set(CMAKE_CUDA_COMPILER ${CUDA_NVCC_EXECUTABLE})
    
    foreach (dir ${CUR_SUB_DIR})
        file(GLOB_RECURSE tmp_srcs ${dir}/*.cu ${dir}/*.cuh ./gpujpeg*.h ./*gpujpeg*.cpp ./*gpujpeg*.c ./*.c ./*.h)
        list(APPEND CUR_CU_SRCS ${tmp_srcs})
    endforeach ()
endif() 
message("CUR_SRCS = ${CUR_SRCS}")
message("CUR_CU_SRCS = ${CUR_CU_SRCS}")

if(CUDA_FOUND)
        add_library(${CUR_LIB} STATIC ${CUR_SRCS} ${CUR_CU_SRCS})
        
        if(WIN32) 
                target_include_directories(${CUR_LIB}
                        PUBLIC
                        ${CUR_INCLUDES}
                        )
        else()
                target_include_directories(${CUR_LIB}
                        PUBLIC
                        ${CUR_INCLUDES}
                        ) 
        endif()

        target_link_libraries(${CUR_LIB}
                PUBLIC
                ${CUDA_LIBRARIES}
                )
        target_compile_definitions(${CUR_LIB} PRIVATE MODULE_NAME="codec")
else()
        add_library(${CUR_LIB} STATIC ${CUR_SRCS})

        if(WIN32)
                target_include_directories(${CUR_LIB}
                        PUBLIC
                        ${CUR_INCLUDES}
                        )
        else()
                target_include_directories(${CUR_LIB}
                        PUBLIC
                        ${CUR_INCLUDES}
                        ) 
        endif()

        target_link_libraries(${CUR_LIB}
                PUBLIC
                )
        target_compile_definitions(${CUR_LIB} PRIVATE MODULE_NAME="codec")
endif() 

set(enable_test false)
if(enable_test)
        message("enable codec test !")
        add_executable(color_test ./test/color_test.cpp)
        target_link_libraries(color_test codec)

        add_executable(jpeg_test ./test/jpeg_test.cpp)
        target_link_libraries(jpeg_test codec)

        add_executable(h265_test ./test/h265_test.cpp)
        target_link_libraries(h265_test codec)
else() 
        message("disable codec test !")
endif(enable_test)
