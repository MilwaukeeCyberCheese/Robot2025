TEMPLATE = app
TARGET = teleopEnabler

CONFIG += console
CONFIG -= qt
CONFIG -= app_bundle

# Specify platform/compiler if needed
win32 {
    QMAKE_CXXFLAGS += -std=c++17
}

# Verify this path is correct
include($$PWD/LibDS/LibDS.pri)

SOURCES += main.cpp