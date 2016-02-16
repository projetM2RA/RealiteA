#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <QButtonGroup>
#include <QCheckBox>

#include <QPushButton>

#include <QComboBox>
#include <QLabel>
#include <QDoubleSpinBox>

#include <QMessageBox>

#include "osgwidget.h"
#include "sideviewosgwidet.h"
#include "webcamdevice.h"
#include "addobjectdialog.h"


#define NBR_DETECT  4
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
    void addObject();
    void updateObjectCharacteristics(int objectID);
    void displayObjectInScene(bool display) { m_mainView->displayObjectInScene(m_objectID, display); }

private:
    void setWindow();
    void connectAll();

    //////////////////////////////////////////////////
    ////////// widgets graphiques ////////////////////
    //////////////////////////////////////////////////

    QButtonGroup* m_detectGroup;
    QCheckBox** m_detectBoxes;

    QPushButton* m_addObjectButton;
    QPushButton* m_calibrateButton;
    QPushButton* m_fullScreenButton;

    OSGWidget* m_mainView;

    QComboBox* m_objectChoiceComboBox;
    QPushButton* m_deleteObjectButton;
    QCheckBox* m_isPrintedBox;
    QSlider** m_objectCharacteristicsSpinSliders;

    SideViewOsgWidet* m_sideView;

    //////////////////////////////////////////////////

    WebcamDevice* m_webcamDevice;
    int m_objectID;
};

#endif // MAINWINDOW_H
