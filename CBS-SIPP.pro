TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
TARGET = CBS_SIPP
SOURCES += main.cpp \
    cbs.cpp \
    config.cpp \
    map.cpp \
    tinyxml2.cpp \
    task.cpp \
    sipp.cpp \
    xml_logger.cpp \
    heuristic.cpp

HEADERS += \
    config.h \
    structs.h \
    cbs.h \
    map.h \
    tinyxml2.h \
    task.h \
    const.h \
    sipp.h \
    xml_logger.h \
    heuristic.h
