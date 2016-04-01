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

    CvSize _frameSize; // TODO  je ne sais pas vraiment à quoi il sert... il est utilisé pour opengl (drawBackground) mais n'est pas initialisé...
    cv::Mat* _cvFrame;
};

#endif // WEBCAMGRAPHICSSCENE_H
