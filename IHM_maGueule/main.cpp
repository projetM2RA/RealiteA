#include "MainWindow.h"
#include <QApplication>


Q_DECLARE_METATYPE(cv::Mat)

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QFile File(":/css/style");
    File.open(QFile::ReadOnly);
    QString StyleSheet = QLatin1String(File.readAll());

    app.setStyleSheet(StyleSheet);

    MainWindow w;
    w.show();

    return app.exec();
}
