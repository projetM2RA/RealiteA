#ifndef ADDOBJECTDIALOG_H
#define ADDOBJECTDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QCheckBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QPushButton>

class AddObjectDialog : public QDialog
{
    Q_OBJECT
public:
    explicit AddObjectDialog(QWidget *parent = 0);

    QString getObjectName() { return m_objectName->text(); }
    QString getObjectPath() { return m_objectPath->text(); }
    bool isMask() { return m_maskBox->isChecked(); }


signals:

public slots:

private slots:
    void updatePath();

private:
    QLineEdit *m_objectName;
    QLineEdit *m_objectPath;
    QCheckBox *m_maskBox;
};

#endif // ADDOBJECTDIALOG_H
