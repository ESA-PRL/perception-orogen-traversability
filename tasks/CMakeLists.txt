include(traversabilityTaskLib)
ADD_LIBRARY(${TRAVERSABILITY_TASKLIB_NAME} SHARED 
    ${TRAVERSABILITY_TASKLIB_SOURCES})
add_dependencies(${TRAVERSABILITY_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${TRAVERSABILITY_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${QT_LIBRARIES}
    ${TRAVERSABILITY_TASKLIB_DEPENDENT_LIBRARIES})
SET_TARGET_PROPERTIES(${TRAVERSABILITY_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${TRAVERSABILITY_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${TRAVERSABILITY_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${TRAVERSABILITY_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${TRAVERSABILITY_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${TRAVERSABILITY_TASKLIB_HEADERS}
    DESTINATION include/orocos/traversability)
