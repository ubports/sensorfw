TEMPLATE     = lib
CONFIG      += plugin

include( ../common-config.pri )

SENSORFW_INCLUDEPATHS = ../..           \
                        ../../include \
                        ../../core   \
                        ../../datatypes \
                        ../../adaptors \
                        ../../filters

DEPENDPATH  += $$SENSORFW_INCLUDEPATHS
INCLUDEPATH += $$SENSORFW_INCLUDEPATHS

include(../common-install.pri)
publicheaders.files += $$HEADERS
target.path = $$PLUGINPATH

INSTALLS += target
