TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
TARGET = CCBS
win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}
INCLUDEPATH += D:/boost_1_75_0
SOURCES += main.cpp \
    cbs.cpp \
    config.cpp \
    map.cpp \
    simplex/columnset.cpp \
    simplex/constraint.cpp \
    simplex/matrix.cpp \
    simplex/objectivefunction.cpp \
    simplex/simplex.cpp \
    simplex/variable.cpp \
    tinyxml2.cpp \
    task.cpp \
    sipp.cpp \
    TO-AA-SIPP/to_aa_sipp.cpp \
    xml_logger.cpp \
    heuristic.cpp

HEADERS += \
    config.h \
    simplex/columnset.h \
    simplex/constraint.h \
    simplex/datamismatchexception.h \
    simplex/divisionbyzeroexception.h \
    simplex/indexoutofboundexception.h \
    simplex/matrix.h \
    simplex/matrixissingularexception.h \
    simplex/matrixnotsquareexception.h \
    simplex/memoryreachedoutexception.h \
    simplex/notavectorexception.h \
    simplex/objectivefunction.h \
    simplex/pilal.h \
    simplex/pilalexceptions.h \
    simplex/simplex.h \
    simplex/simplexexceptions.h \
    simplex/sizemismatchexception.h \
    simplex/variable.h \
    structs.h \
    cbs.h \
    map.h \
    tinyxml2.h \
    task.h \
    const.h \
    sipp.h \
    TO-AA-SIPP/to_aa_sipp.h \
    TO-AA-SIPP/states_container.h \
    xml_logger.h \
    heuristic.h \
    TO-AA-SIPP/lineofsight.h
