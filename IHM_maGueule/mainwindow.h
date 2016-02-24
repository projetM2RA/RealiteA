#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <QMenu>
#include <QMenuBar>

#include <QActionGroup>
#include <QAction>

#include <QPushButton>

#include <QComboBox>
#include <QLabel>
#include <QDoubleSpinBox>

#include <QMessageBox>

#include <QShortcut>

#include "OsgWidget.h"
#include "SideViewOsgWidget.h"
#include "WebcamDevice.h"
#include "AddObjectDialog.h"


#define NBR_CHARACTERISTICS  10

enum { noDetect = 0, chehra = 1, chess = 2, QR = 3 };
enum { sizeX = 0, sizeY = 1, sizeZ = 2, rotX = 3, rotY = 4, rotZ = 5, transX = 6, transY = 7, transZ = 8, alpha = 9 };


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void updateCam() { _backgroundImage->dirty(); _mainView->repaint(); _sideView->repaint(); _fullScreenView->repaint(); }
    void displayObjects(bool removeObjects) { _objectsList[0]->setNodeMask(!removeObjects); }
    void displayObjectsInSideView(bool removeObjects) { _objectsList2[0]->setNodeMask(!removeObjects); _isPrintedBox->setChecked(!removeObjects); }
    void start();
    void calibrateCamera();
    void addObject();
    void updateObjectCharacteristics(int objectID);
    void updateDetectMode();
    void switchInput();
    void displayObjectInScene(bool display);
    void displayObjectInSideView(bool display);
    void displayFullScreen();
    void updateSceneRT(cv::Mat rotVec, cv::Mat tvecs);

private:
    void setFirstWindow();
    void setMainWindow();
    void connectAll();
    void setShortcuts();
    void createFullScreenWidget();
    void initObjectsList();

    //////////////////////////////////////////////////
    ////////// widgets graphiques ////////////////////
    //////////////////////////////////////////////////


    QAction* _startAction;
    QAction* _addObjectAction;
    QAction* _calibrateAction;
    QAction* _optionsAction;
    QAction* _fullScreenAction;

    QActionGroup* _detectGroup;
    QAction** _detectActions;

    QActionGroup* _webcamGroup;
    QAction** _webcamActions;
    QAction* _videoAction;

    OSGWidget* _mainView;

    QComboBox* _objectChoiceComboBox;
    QPushButton* _deleteObjectButton;
    QCheckBox* _isPrintedBox;
    QCheckBox* _isPrintedBox2;
    QSlider** _objectCharacteristicsSpinSliders;

    SideViewOsgWidet* _sideView;


    //////////////////////////////////////////////////
    ////////// widget full screen ////////////////////
    //////////////////////////////////////////////////


    QWidget* _fullScreenWidget;
    OSGWidget* _fullScreenView;


    //////////////////////////////////////////////////
    ////////// raccourcis clavier ////////////////////
    //////////////////////////////////////////////////

    QShortcut* _fullScreenShortcut;
    QShortcut* _fullScreenShortcut2;
    QShortcut* _leaveFullScreen;
    QShortcut* _pauseShortcut;
    QShortcut* _pauseShortcut2;

    //////////////////////////////////////////////////

    WebcamDevice* _webcamDevice;
    std::vector<Our3DObject*> _objectsList;
    osg::MatrixTransform *_mainMat;
    std::vector<Our3DObject*> _objectsList2;
    osg::MatrixTransform *_mainMat2;
    osg::Image* _backgroundImage;
    double _corrector;
    int _objectID;
    int _nbrCam;
    bool _fullScreen;
};

#endif // MAINWINDOW_H
