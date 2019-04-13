TARGET = iiosensorsadaptor

HEADERS += iioadaptor.h \
           iioadaptorplugin.h

SOURCES += iioadaptor.cpp \
           iioadaptorplugin.cpp

CONFIG += qt debug warn_on link_prl link_pkgconfig plugin

CONFIG += link_pkgconfig
PKGCONFIG += libudev
LIBS += -ludev

include( ../adaptor-config.pri )
