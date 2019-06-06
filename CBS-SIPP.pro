TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
TARGET = C-CBS
SOURCES += main.cpp \
    cbs.cpp \
    graph.cpp \
    heuristic.cpp \
    sipp.cpp \
    task.cpp \
    tinyxml2.cpp \
    xml_logger.cpp


HEADERS += \
    cbs.h \
    const.h \
    graph.h \
    heuristic.h \
    sipp.h \
    structs.h \
    task.h \
    tinyxml2.h \
    xml_logger.h

