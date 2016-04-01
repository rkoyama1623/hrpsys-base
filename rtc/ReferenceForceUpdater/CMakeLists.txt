set(comp_sources ReferenceForceUpdater.cpp ReferenceForceUpdaterService_impl.cpp)
set(libs hrpsysBaseStub)
add_library(ReferenceForceUpdater SHARED ${comp_sources})
target_link_libraries(ReferenceForceUpdater ${libs})
set_target_properties(ReferenceForceUpdater PROPERTIES PREFIX "")

add_executable(ReferenceForceUpdaterComp ReferenceForceUpdaterComp.cpp ${comp_sources})
target_link_libraries(ReferenceForceUpdaterComp ${libs})

set(target ReferenceForceUpdater ReferenceForceUpdaterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
