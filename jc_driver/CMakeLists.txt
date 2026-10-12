find_package(Threads REQUIRED)
find_package(Boost REQUIRED)
find_library(READLINE_LIBRARY readline REQUIRED)
set(CMAKE_CXX_STANDARD 17)
file(GLOB SOEM_SOURCES thirdparty/soem/soem/*.c)
add_library(soem STATIC ${SOEM_SOURCES}
    thirdparty/soem/osal/linux/osal.c thirdparty/soem/oshw/linux/nicdrv.c thirdparty/soem/oshw/linux/oshw.c)
target_include_directories(soem PUBLIC thirdparty/soem/soem thirdparty/soem/osal
    thirdparty/soem/osal/linux thirdparty/soem/oshw/linux)
target_link_libraries(soem PUBLIC Threads::Threads rt)
add_library(cpp-readline STATIC thirdparty/cpp-readline/src/Console.cpp)
target_include_directories(cpp-readline PUBLIC thirdparty/cpp-readline/src)
target_link_libraries(cpp-readline PUBLIC ${READLINE_LIBRARY})
add_library(j3_driver STATIC
    utils/utils.c utils/jc_log.cpp services/canopen/canopen.c
    services/sharemem/SharedMemory.cpp services/ethercat/transmit.cpp
    services/ethercat/ethercat_msg_pool.cpp services/ethercat/ethercat_canfd_msg_pool.cpp
    drivers/m18/m18.cpp application/application.cpp application/app_comm.cpp
    application/triple_map.cpp application/jc01/jc01.cpp)
target_include_directories(j3_driver PUBLIC ${Boost_INCLUDE_DIRS} utils services/comm
    services/canopen services/sharemem services/ethercat drivers/m18 application application/jc01)
target_compile_definitions(j3_driver PUBLIC DRIVER_M18_ENABLE APPLICATION_JC01 SINGLE_J3_M18=1)
target_link_libraries(j3_driver PUBLIC soem cpp-readline Threads::Threads rt m)
add_executable(motor_test tests/motor_test/main.cpp)
target_link_libraries(motor_test PRIVATE j3_driver)
if(BUILD_TESTING)
    add_executable(driver_contracts tests/offline/driver_contracts.cpp application/app_comm.cpp
        application/triple_map.cpp services/canopen/canopen.c)
    target_include_directories(driver_contracts BEFORE PRIVATE tests/offline)
    target_compile_options(driver_contracts PRIVATE -ffunction-sections -fdata-sections)
    target_link_options(driver_contracts PRIVATE -Wl,--gc-sections)
    target_include_directories(driver_contracts PRIVATE $<TARGET_PROPERTY:j3_driver,INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:soem,INTERFACE_INCLUDE_DIRECTORIES>)
    target_compile_definitions(driver_contracts PRIVATE $<TARGET_PROPERTY:j3_driver,COMPILE_DEFINITIONS>)
    target_link_libraries(driver_contracts PRIVATE cpp-readline Threads::Threads rt m)
    add_test(NAME driver_contracts COMMAND driver_contracts)
endif()
file(COPY tools/start_driver.sh tools/send_command.sh tools/driver_session.exp
    DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
