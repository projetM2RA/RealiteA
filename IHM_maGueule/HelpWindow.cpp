#include "HelpWindow.h"

HelpWindow::HelpWindow(QWidget *parent) :
    QDialog(parent)
{
    QHelpEngine* helpEngine = new QHelpEngine("../rsc/help/help.qhc");
    helpEngine->setupData();

    QTabWidget* tWidget = new QTabWidget;
    tWidget->setMaximumWidth(200);
    tWidget->addTab((QWidget*)helpEngine->contentWidget(), "Contents");
    tWidget->addTab((QWidget*)helpEngine->indexWidget(), "Index");

    HelpBrowser *textViewer = new HelpBrowser(helpEngine);
    textViewer->setSource(
                QUrl("qthelp://aron.help/Howto/intro.html"));
    connect((QWidget*)helpEngine->contentWidget(),
            SIGNAL(linkActivated(QUrl)),
            textViewer, SLOT(setSource(QUrl)));

    connect((QWidget*)helpEngine->indexWidget(),
            SIGNAL(linkActivated(QUrl, QString)),
            textViewer, SLOT(setSource(QUrl)));

    QSplitter *horizSplitter = new QSplitter(Qt::Horizontal);
    horizSplitter->insertWidget(0, tWidget);
    horizSplitter->insertWidget(1, textViewer);

    QHBoxLayout* hl = new QHBoxLayout();
    hl->addWidget(horizSplitter);

    this->setLayout(hl);

    this->setWindowTitle("Tutorial");
    this->setWindowIcon(QIcon(":/icons/icon"));
    this->setWindowFlags(this->windowFlags() & (~Qt::WindowContextHelpButtonHint));
    this->resize(1500, 750);
}
