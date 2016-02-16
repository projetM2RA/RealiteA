#-------------------------------------------------
#
# Project created by QtCreator 2016-01-26T18:45:11
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = projetRA_IHM
TEMPLATE = app

#OpenCV
INCLUDEPATH += E:\Logiciels\Prog\opencv_2_4_11\opencv\build\include

LIBS += -L"E:\Logiciels\Prog\opencv_2_4_11\opencv\build\x64\vc11\lib"
win32:CONFIG(debug, debug|release): LIBS += -lopencv_calib3d2411d -lopencv_contrib2411d \
    -lopencv_core2411d -lopencv_features2d2411d -lopencv_flann2411d -lopencv_gpu2411d \
    -lopencv_highgui2411d -lopencv_imgproc2411d -lopencv_legacy2411d -lopencv_ml2411d \
    -lopencv_nonfree2411d -lopencv_objdetect2411d -lopencv_ocl2411d -lopencv_photo2411d \
    -lopencv_stitching2411d -lopencv_superres2411d -lopencv_ts2411d -lopencv_video2411d \
    -lopencv_videostab2411d
win32:CONFIG(release, debug|release): LIBS += -lopencv_calib3d2411 \
    -lopencv_core2411 -lopencv_features2d2411 -lopencv_imgproc2411 \
    -lopencv_objdetect2411 -lopencv_highgui2411
##

#OpenScenGraph
INCLUDEPATH += E:\Logiciels\Prog\OpenSceneGraph-3.4.0\include

LIBS += -L"E:\Logiciels\Prog\OpenSceneGraph-3.4.0\lib"
win32:CONFIG(debug, debug|release): LIBS += -lOpenThreadsd -losgAnimationd -losgd \
    -losgDBd -losgFXd -losgGAd -losgManipulatord -losgParticled -losgPresentationd -losgShadowd \
    -losgSimd -losgTerraind -losgTextd -losgUId -losgUtild -losgViewerd  -losgVolumed -losgWidgetd
win32:CONFIG(release, debug|release): LIBS += -lOpenThreads -losg \
    -losgDB -losgGA -losgViewer
##

#Chehra
INCLUDEPATH += E:\Logiciels\Prog\Chehra

LIBS += -L"E:\Logiciels\Prog\Chehra"
win32:CONFIG(debug, debug|release): LIBS += -lChehra_d
win32:CONFIG(release, debug|release): LIBS += -lChehra_r
##


SOURCES += main.cpp\
        mainwindow.cpp \
    osgwidget.cpp \
    our3dobject.cpp \
    webcamdevice.cpp \
    addobjectdialog.cpp \
    sideviewosgwidet.cpp

HEADERS  += mainwindow.h \
    osgwidget.h \
    our3dobject.h \
    webcamdevice.h \
    addobjectdialog.h \
    sideviewosgwidet.h

RESOURCES += \
    rscImg.qrc
