#ifndef WEBCAMGRAPHICSSCENE_H
#define WEBCAMGRAPHICSSCENE_H

#include <QtWidgets>
#include <QGLWidget>
#include <opencv2/opencv.hpp>

class WebcamGraphicsScene : public QGraphicsScene
{

    Q_OBJECT

public:
    WebcamGraphicsScene(cv::Mat* cam, QObject* parent = 0);
    ~WebcamGraphicsScene();

    void drawBackground(QPainter *, const QRectF &);
    cv::Mat getFrame() { return *_cvFrame; }

private:
    void deleteItems();

    CvSize _frameSize; // TODO  je ne sais pas vraiment Ã  quoi il sert... il est utilisÃ© pour opengl (drawBackground) mais n'est pas initialisÃ©...
    cv::Mat* _cvFrame;
};

#endif // WEBCAMGRAPHICSSCENE_H
