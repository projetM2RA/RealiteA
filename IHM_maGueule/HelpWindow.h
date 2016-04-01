////////////////////////////////////////////////////////////
//                                                        //
//  Fenetre de dialogue d'aide qui va présenter les       //
//  principales caractéristiques du logicel et comment    //
//  s'en servir.                                          //
//  Pour plus de détails, voir :                          //
// http://www.walletfox.com/course/qhelpengineexample.php //
//                                                        //
////////////////////////////////////////////////////////////


#ifndef HELPWINDOW_H
#define HELPWINDOW_H

#include <QDialog>
#include <QHelpEngine>
#include <QTabWidget>
#include <QSplitter>
#include <QHBoxLayout>

#include "HelpBrowser.h"

class HelpWindow : public QDialog
{
    Q_OBJECT
public:
    explicit HelpWindow(QWidget *parent = 0);

signals:

public slots:

};

#endif // HELPWINDOW_H
