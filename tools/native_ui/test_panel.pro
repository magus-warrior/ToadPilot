QT += widgets network testlib
CONFIG += c++17 testcase
CONFIG -= debug
QMAKE_CXX = clang++
TARGET = test-panel
INCLUDEPATH += ../..
SOURCES += test_panel.cc ../../selfdrive/ui/qt/offroad/toadpilot_panel.cc
HEADERS += ../../selfdrive/ui/qt/offroad/toadpilot_panel.h
