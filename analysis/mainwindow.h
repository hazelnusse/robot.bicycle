#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QFuture>
#include <QFutureWatcher>
#include <QMainWindow>
#include <QListWidget>
#include <QString>
#include <QVector>
#include <QMutex>
#include "qcustomplot.h"
#include "sample.pb.h"

namespace gui {

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  explicit MainWindow(const QVector<QString> & data_filenames, QWidget *parent = 0);
  ~MainWindow();
  
private slots:
    void axisLabelDoubleClick(QCPAxis * axis, QCPAxis::SelectablePart part);
    void titleDoubleClick(QMouseEvent *, QCPPlotTitle * title);
    void graphClicked(QCPAbstractPlottable * plottable);
    void legendDoubleClick(QCPLegend * legend, QCPAbstractLegendItem * item);
    void mousePress();
    void mouseWheel();
    void selectionChanged();
    void savePDF();
    void setup_plot();
    void populate_listwidget();
    void selectedFileChanged();

private:
    void setup_layout();
    QCustomPlot * plot_;
    QList<QString> data_filenames_;
    QMap<QString, QVector<sample::Sample>> data_set_;
    QMutex mutex;

    QListWidget * listwidget_;
    QFuture<QString> future_;
    QFutureWatcher<QString> watcher_;

    QLineEdit * width_edit_, * height_edit_;

    QVector<QString> fields_;
    QVector<QString> selected_fields_;
    QString selected_file_;
};

} // namespace gui

#endif // MAINWINDOW_H

