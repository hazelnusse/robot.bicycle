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
#include "datawrangler.h"

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
    void selectedFieldsChanged(int state);
    void selectedFileChanged();

private:
    void setup_layout();
    QCustomPlot * plot_;
    QList<QString> data_filenames_;
    // Map filename to vector of protobuf Sample messages
    QMap<QString, QVector<sample::Sample>> proto_messages_;
    // Map filename and fieldname to QVector of doubles (time series data)
    // fieldname is roughly the protobuf message field name, it also is used in
    // the main gui for checkboxes
    QMap<QString, QMap<QString, QVector<double>>> time_series_;

    QListWidget * listwidget_;
    QFuture<QString> future_;
    QFutureWatcher<QString> watcher_;

    QLineEdit * width_edit_, * height_edit_;

    QVector<QString> fields_;
    // QList<QString> selected_fields_;
    QString selected_file_;

    QMap<QString, QCPGraph *> selected_fields_;

    gui::DataWrangler dw_;
};

} // namespace gui

#endif // MAINWINDOW_H

