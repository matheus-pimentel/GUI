#-------------------------------------------------
#
# Project created by QtCreator 2018-01-24T13:52:17
#
#-------------------------------------------------

QT       += core gui
QT += 3dcore 3drender 3dinput 3dextras

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GUI
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    utils.cpp \
    quad.cpp \
    controller.cpp \
    scenemodifier.cpp \
    pso.cpp

HEADERS += \
        mainwindow.h \
    utils.h \
    quad.h \
    controller.h \
    scenemodifier.h \
    pso.h

FORMS += \
        mainwindow.ui
