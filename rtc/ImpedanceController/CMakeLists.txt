set(comp_sources ImpedanceController.cpp ImpedanceControllerService_impl.cpp JointPathEx.cpp RatsMatrix.cpp ImpedanceOutputGenerator.h ObjectTurnaroundDetector.h InternalForceSeparator.cpp ../TorqueFilter/IIRFilter.cpp)
if(USE_QPOASES)
  link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/bin)
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/include)
  add_definitions(-DUSE_QPOASES)
endif()
if(USE_QPOASES)
  set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub qpOASES)
else()
  set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
endif()

add_library(ImpedanceController SHARED ${comp_sources})
target_link_libraries(ImpedanceController ${libs})
set_target_properties(ImpedanceController PROPERTIES PREFIX "")

add_executable(ImpedanceControllerComp ImpedanceControllerComp.cpp ${comp_sources})
target_link_libraries(ImpedanceControllerComp ${libs})

add_executable(testImpedanceOutputGenerator testImpedanceOutputGenerator.cpp ImpedanceOutputGenerator.h RatsMatrix.cpp)
target_link_libraries(testImpedanceOutputGenerator ${libs})
add_executable(testObjectTurnaroundDetector testObjectTurnaroundDetector.cpp ObjectTurnaroundDetector.h ../TorqueFilter/IIRFilter.cpp)
target_link_libraries(testObjectTurnaroundDetector ${libs})
add_executable(testInternalForceSeparator testInternalForceSeparator.cpp InternalForceSeparator.cpp)
target_link_libraries(testInternalForceSeparator ${libs})

add_library(JointPathExC SHARED JointPathExC.cpp JointPathEx.cpp)
target_link_libraries(JointPathExC ${libs})

set(target ImpedanceController ImpedanceControllerComp testImpedanceOutputGenerator testObjectTurnaroundDetector testInternalForceSeparator JointPathExC)

add_test(testImpedanceOutputGeneratorTest0 testImpedanceOutputGenerator --test0 --use-gnuplot false)
add_test(testImpedanceOutputGeneratorTest1 testImpedanceOutputGenerator --test1 --use-gnuplot false)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

