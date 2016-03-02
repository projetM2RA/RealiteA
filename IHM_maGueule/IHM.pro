#-------------------------------------------------
#
# Project created by QtCreator 2016-01-26T18:45:11
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = IHM
TEMPLATE = app

#OpenCV
INCLUDEPATH += C:\Libs\opencv\build\include

LIBS += -L"C:\Libs\opencv\build\x64\vc11\lib"
win32:CONFIG(debug, debug|release): LIBS += -lopencv_calib3d2411d -lopencv_contrib2411d \
    -lopencv_core2411d -lopencv_features2d2411d -lopencv_flann2411d -lopencv_gpu2411d \
    -lopencv_highgui2411d -lopencv_imgproc2411d -lopencv_legacy2411d -lopencv_ml2411d \
    -lopencv_nonfree2411d -lopencv_objdetect2411d -lopencv_ocl2411d -lopencv_photo2411d \
    -lopencv_stitching2411d -lopencv_superres2411d -lopencv_ts2411d -lopencv_video2411d \
    -lopencv_videostab2411d
win32:CONFIG(release, debug|release): LIBS += -lopencv_calib3d2411 -lopencv_contrib2411 \
    -lopencv_core2411 -lopencv_features2d2411 -lopencv_flann2411 -lopencv_gpu2411 \
    -lopencv_highgui2411 -lopencv_imgproc2411 -lopencv_legacy2411 -lopencv_ml2411 \
    -lopencv_nonfree2411 -lopencv_objdetect2411 -lopencv_ocl2411 -lopencv_photo2411 \
    -lopencv_stitching2411 -lopencv_superres2411 -lopencv_ts2411 -lopencv_video2411 \
    -lopencv_videostab2411
##

#OpenScenGraph
INCLUDEPATH += C:\Libs\OpenSceneGraph-3.4.0\include

LIBS += -L"C:\Libs\OpenSceneGraph-3.4.0\lib"
win32:CONFIG(debug, debug|release): LIBS += -lOpenThreadsd -losgAnimationd -losgd \
    -losgDBd -losgFXd -losgGAd -losgManipulatord -losgParticled -losgPresentationd -losgShadowd \
    -losgSimd -losgTerraind -losgTextd -losgUId -losgUtild -losgViewerd  -losgVolumed -losgWidgetd
win32:CONFIG(release, debug|release): LIBS += -lOpenThreads -losgAnimation -losg \
    -losgDB -losgFX -losgGA -losgManipulator -losgParticle -losgPresentation -losgShadow \
    -losgSim -losgTerrain -losgText -losgUI -losgUtil -losgViewer  -losgVolume -losgWidget
##

#Chehra
INCLUDEPATH += C:\Libs\Chehra

LIBS += -L"C:\Libs\Chehra"
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
