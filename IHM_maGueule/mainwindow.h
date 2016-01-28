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

#define NBR_DETECT  3
#define NBR_CHARACTERISTICS  9

enum { chehra = 0, chess = 1, QR = 2 };
enum { sizeX = 0, sizeY = 1, sizeZ = 2, rotX = 3, rotY = 4, rotZ = 5, transX = 6, transY = 7, transZ = 8 };


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    void setWindow();

    QCheckBox** m_detectBoxes;

    QCheckBox* m_occlusionBox;

    QPushButton* m_addObjectButton;
    QPushButton* m_3DSlicerButton;
    QPushButton* m_fullScreenButton;



    QComboBox* m_objectChoiceComboBox;
    QDoubleSpinBox** m_objectCharacteristicsSpinBoxes;

    //NotreClasseDAffichage
};

#endif // MAINWINDOW_H
