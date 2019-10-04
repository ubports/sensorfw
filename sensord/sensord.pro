QT += network

TEMPLATE = app

include( ../common-config.pri )
TARGET = sensorfwd

CONFIG += console \
          link_pkgconfig

PKGCONFIG += libsystemd deviceinfo

SENSORFW_INCLUDEPATHS = .. \
                        ../include \
                        ../filters \
                        ../datatypes \
                        ../core \
                        ../sensors/magnetometersensor

DEPENDPATH += $$SENSORFW_INCLUDEPATHS
INCLUDEPATH += $$SENSORFW_INCLUDEPATHS

QMAKE_LIBDIR_FLAGS += -L../datatypes \
                      -L../core

TARGET_H.path += /usr/include/sensord-qt5

SOURCES += main.cpp \
           parser.cpp \
           calibrationhandler.cpp

HEADERS += parser.h \
           calibrationhandler.h

contextprovider {
    DEFINES += PROVIDE_CONTEXT_INFO
    PKGCONFIG += contextprovider-1.0
}

TARGET_H.files = $$HEADERS
target.path = /usr/sbin/

INSTALLS += target \
            TARGET_H

include(../common.pri)
