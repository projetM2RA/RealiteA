#include "webcamgraphicsscene.h"

WebcamGraphicsScene::WebcamGraphicsScene(cv::Mat *cam, QObject *parent) :
    QGraphicsScene(parent)
{
    _frameSize.width = cam->cols;
    _frameSize.height = cam->rows;

    _cvFrame = cam;

    this->setSceneRect(0,0,_frameSize.width,_frameSize.height);
    this->update();
}

WebcamGraphicsScene::~WebcamGraphicsScene()
{
    deleteItems();
}

void WebcamGraphicsScene::drawBackground(QPainter *, const QRectF &)
{
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glScalef(1.0, 1.0, 1.0);
    glTranslatef(0.0, 0.0, 0.0);
    glOrtho(0, _frameSize.width, _frameSize.height, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glEnable(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D( GL_TEXTURE_2D, 0, 4, _cvFrame->cols, _cvFrame->rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, _cvFrame->data );
    glBegin(GL_QUADS);

//    glTexCoord2f(0,0); glVertex2f(0,0);
//    glTexCoord2f(0,1); glVertex2f(0,_frameSize.height);
//    glTexCoord2f(1,1); glVertex2f(_frameSize.width,_frameSize.height);
//    glTexCoord2f(1,0); glVertex2f(_frameSize.width,0);

    glTexCoord2f(0,0); glVertex2f(_frameSize.width,0);
    glTexCoord2f(0,1); glVertex2f(_frameSize.width,_frameSize.height);
    glTexCoord2f(1,1); glVertex2f(0,_frameSize.height);
    glTexCoord2f(1,0); glVertex2f(0,0);

    glEnd();
    glDisable(GL_TEXTURE_2D);
    glFlush();
}


// public slots

void WebcamGraphicsScene::deleteItems(){
    QList<QGraphicsItem *> list_items = this->items();
    QList<QGraphicsItem *>::Iterator it = list_items.begin();
    for ( ; it != list_items.end(); ++it )
    {
        if ( *it )
        {
            this->removeItem(*it);
            delete *it;
        }
    }
}
