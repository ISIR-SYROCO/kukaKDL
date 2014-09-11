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
