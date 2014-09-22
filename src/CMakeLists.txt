# Copyright 2014 ISIR-CNRS
# Authors: Sovannara HAK

ADD_LIBRARY(
    ${PROJECT_NAME}
    ${LIB_TYPE}
    kukakdl.cpp
)

TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}
    ${PC_OROCOS_KDL_LIBRARIES}
)

INSTALL(
    TARGETS ${PROJECT_NAME}
    ${install_type} DESTINATION lib
)

IF(PC_EIGEN_LGSM_FOUND AND PC_ORC_FRAMEWORK_FOUND)
    INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/include
        ${PC_OROCOS_KDL_INCLUDE_DIRS}
        ${PC_EIGEN_LGSM_INCLUDE_DIRS}
        ${PC_ORC_FRAMEWORK_INCLUDE_DIRS}
    )
    LINK_DIRECTORIES(
        ${PC_OROCOS_KDL_LIBRARY_DIRS}
        ${PC_ORC_FRAMEWORK_LIBRARY_DIRS}
    )

    ADD_LIBRARY(
        orckukakdl
        ${LIB_TYPE}
        orckukakdl.cpp
    )

    TARGET_LINK_LIBRARIES(
        orckukakdl
        ${PROJECT_NAME}
        ${PC_OROCOS_KDL_LIBRARIES}
        ${PC_ORC_FRAMEWORK_LIBRARIES}
    )

    INSTALL(
        TARGETS orckukakdl
        ${install_type} DESTINATION lib
    )

ENDIF(PC_EIGEN_LGSM_FOUND AND PC_ORC_FRAMEWORK_FOUND)

