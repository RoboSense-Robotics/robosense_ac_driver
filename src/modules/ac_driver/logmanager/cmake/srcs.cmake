#========================
# libs
#========================

set(CUR_SRCS "")
set(CUR_INCLUDES "include")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

add_library(${CUR_LIB} STATIC
        ${CUR_SRCS}
        )

# message("spdlog ===> ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/spdlog")
target_include_directories(${CUR_LIB}
        PUBLIC
        ${CUR_INCLUDES}
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/spdlog
        )
        
set(enable_test false)
if(enable_test)
        message("enable log test !")
        add_executable(logmanager_test ./test/logmanager_test.cpp)
        target_link_libraries(logmanager_test log)
else() 
        message("disable log test !")
endif(enable_test) 

#=============================
# install
#=============================

install(TARGETS ${CUR_LIB}
        LIBRARY DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
        ARCHIVE DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
        COMPONENT release
        )
