#-------------------------------------------------
#
# Project created by QtCreator 2016-01-26T18:45:11
#
#-------------------------------------------------

QT       += core gui opengl xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = projetRA_IHM
TEMPLATE = app

#OpenCV
INCLUDEPATH += E:\Logiciels\Prog\opencv_2_4_11\opencv\build\include

LIBS += -L"E:\Logiciels\Prog\opencv_2_4_11\opencv\build\x64\vc11\lib"

win32:CONFIG(debug, debug|release): LIBS += -lopencv_calib3d2411d \
    -lopencv_core2411d -lopencv_features2d2411d -lopencv_highgui2411d \
    -lopencv_imgproc2411d -lopencv_objdetect2411d -lopencv_video2411d \
    -lopencv_nonfree2411d -lopencv_flann2411d

win32:CONFIG(release, debug|release): LIBS += -lopencv_calib3d2411 \
    -lopencv_core2411 -lopencv_features2d2411 -lopencv_imgproc2411 \
    -lopencv_objdetect2411 -lopencv_highgui2411 -lopencv_video2411 \
    -lopencv_nonfree2411 -lopencv_flann2411
##

#OpenScenGraph
INCLUDEPATH += E:\Logiciels\Prog\OpenSceneGraph-3.4.0\include

LIBS += -L"E:\Logiciels\Prog\OpenSceneGraph-3.4.0\lib"
win32:CONFIG(debug, debug|release): LIBS += -lOpenThreadsd -losgd \
    -losgDBd -losgGAd -losgViewerd
win32:CONFIG(release, debug|release): LIBS += -lOpenThreads -losg \
    -losgDB -losgGA -losgViewer
##

#Chehra
INCLUDEPATH += E:\Logiciels\Prog\Chehra

LIBS += -L"E:\Logiciels\Prog\Chehra"
win32:CONFIG(debug, debug|release): LIBS += -lChehra_d
win32:CONFIG(release, debug|release): LIBS += -lChehra_r
##


SOURCES += \
    SideViewOsgWidget.cpp \
    OptionsDialog.cpp \
    AddObjectDialog.cpp \
    CalibrateDialog.cpp \
    main.cpp \
    MainWindow.cpp \
    OptionsDialog.cpp \
    OsgWidget.cpp \
    SideViewOsgWidget.cpp \
    WebcamDevice.cpp \
    WebcamGraphicsScene.cpp \
    Our3DObject.cpp \
    MatrixDialog.cpp

HEADERS  += \
    SideViewOsgWidget.h \
    OptionsDialog.h \
    AddObjectDialog.h \
    CalibrateDialog.h \
    MainWindow.h \
    OptionsDialog.h \
    OsgWidget.h \
    SideViewOsgWidget.h \
    WebcamDevice.h \
    WebcamGraphicsScene.h \
    WebcamGraphicsView.h \
    Our3DObject.h \
    MatrixDialog.h

RESOURCES += \
    rsc.qrc \
