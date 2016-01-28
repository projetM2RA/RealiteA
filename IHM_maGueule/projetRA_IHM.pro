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
INCLUDEPATH += D:\logiciels\prog\opencv_2_4_11\opencv\build\include

LIBS += -L"D:\logiciels\prog\opencv_2_4_11\opencv\build\x64\vc11\lib"
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
INCLUDEPATH += D:\logiciels\prog\OpenSceneGraph-3.4.0\include

LIBS += -L"D:\logiciels\prog\OpenSceneGraph-3.4.0\lib"
win32:CONFIG(debug, debug|release): LIBS += -lOpenThreadsd -losgAnimationd -losgd \
    -losgDBd -losgFXd -losgGAd -losgManipulatord -losgParticled -losgPresentationd -losgShadowd \
    -losgSimd -losgTerraind -losgTextd -losgUId -losgUtild -losgViewerd  -losgVolumed -losgWidgetd
win32:CONFIG(release, debug|release): LIBS += -lOpenThreads -losgAnimation -losg \
    -losgDB -losgFX -losgGA -losgManipulator -losgParticle -losgPresentation -losgShadow \
    -losgSim -losgTerrain -losgText -losgUI -losgUtil -losgViewer  -losgVolume -losgWidget
##

#Chehra
INCLUDEPATH += D:\logiciels\prog\Chehra

LIBS += -L"D:\logiciels\prog\Chehra"
win32:CONFIG(debug, debug|release): LIBS += -lChehra_d
win32:CONFIG(release, debug|release): LIBS += -lChehra_r
##


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h
