#========================
# libs
#========================

set(CUR_SRCS "")
set(CUR_INCLUDES "include")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)
LIST(APPEND CUR_SUB_DIR rs_driver/src/rs_driver)

foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

set(THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/rs_driver/thirdparty)
#========================
# libusb 
#========================
include_directories(${THIRD_PARTY_DIR}/libusb) 
include_directories(${THIRD_PARTY_DIR}/libusb/libusb) 
add_subdirectory(${THIRD_PARTY_DIR}/libusb)

#
# libuvc 
#
include_directories(${THIRD_PARTY_DIR}/libuvc) 
include_directories(${THIRD_PARTY_DIR}/libuvc/include)
add_subdirectory(${THIRD_PARTY_DIR}/libuvc)

#
# librsalgo_ac2  
# 
if(ENABLE_SUPPORT_ALGORITHM)
        if(WIN32)
                message(FATAL_ERROR "Sytem Type: Windows librsalgo_ac2 Not Support")
        elseif(UNIX AND NOT APPLE)      
                add_library(rsalgo_ac2 SHARED IMPORTED GLOBAL)
                set_target_properties(rsalgo_ac2 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${THIRD_PARTY_DIR}/librsalgo_ac2/include)     
                if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")       
                        set_target_properties(rsalgo_ac2 PROPERTIES
                                IMPORTED_IMPLIB_RELEASE ${THIRD_PARTY_DIR}/librsalgo_ac2/lib/linux-x86_64/librsalgo_ac2-static.a
                                IMPORTED_LOCATION_RELEASE ${THIRD_PARTY_DIR}/librsalgo_ac2/lib/linux-x86_64/librsalgo_ac2-static.a
                                IMPORTED_IMPLIB_DEBUG ${THIRD_PARTY_DIR}/librsalgo_ac2/lib/linux-x86_64/librsalgo_ac2-static.a
                                IMPORTED_LOCATION_DEBUG ${THIRD_PARTY_DIR}/librsalgo_ac2/lib/linux-x86_64/librsalgo_ac2-static.a
                        )
                else()
                        message(FATAL_ERROR "System Type: Linux No x86_64 librsalgo_ac2 Not Support !")
                endif()	
        elseif(APPLE)
                message(FATAL_ERROR "Sytem Type: Apple librsalgo_ac2 Not Support !")
        else()
                message(FATAL_ERROR "Not Support System Type !")
        endif()
endif(ENABLE_SUPPORT_ALGORITHM)

add_library(${CUR_LIB} STATIC ${CUR_SRCS})

target_include_directories(${CUR_LIB}
        PUBLIC
        ${CUR_INCLUDES}
        ./rs_driver/src
        ${THIRD_PARTY_DIR}/libusb
        ${CMAKE_CURRENT_BINARY_DIR}/../devicemanager/rs_driver/thirdparty/libuvc
        )
        
if(ENABLE_SUPPORT_ALGORITHM)
        target_link_libraries(${CUR_LIB}
                PUBLIC
                usb-ac-static 
                uvc-ac-static  
                rsalgo_ac2
        )
else() 
        target_link_libraries(${CUR_LIB}
                PUBLIC
                usb-ac-static 
                uvc-ac-static  
        )        
endif() 

set(enable_test false)
if(enable_test)
        message("enable device test !")
        add_executable(devicemanager_test ./test/devicemanager_test.cpp)
        target_link_libraries(devicemanager_test device)

        add_executable(devicemanager_test2 ./test/devicemanager_test2.cpp)
        target_link_libraries(devicemanager_test2 device)

        add_executable(devicemanager_test3 ./test/devicemanager_test3.cpp)
        target_link_libraries(devicemanager_test3 device)
else() 
        message("disable device test !")
endif(enable_test) 

#=============================
# install
#=============================

install(TARGETS ${CUR_LIB}
        LIBRARY DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
        ARCHIVE DESTINATION ${HYPER_VISION_DEV_LIB_PATH}
        COMPONENT release
        )
