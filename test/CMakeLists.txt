# Copyright 2014 ISIR-CNRS
# Authors: Sovannara HAK

ADD_EXECUTABLE(testkukakdl
    testkukakdl.cpp
)

TARGET_LINK_LIBRARIES(
    testkukakdl
    ${PROJECT_NAME}
)
