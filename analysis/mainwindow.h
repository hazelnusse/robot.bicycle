#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QFuture>
#include <QFutureWatcher>
#include <QMainWindow>
#include <QString>
#include <QVector>
#include <QMap>
#include "qcustomplot.h"
#include "sample.pb.h"
#include "datawrangler.h"
#include "run_meta_data.h"

class QDoubleSpinBox;
class QListWidget;
class QLineEdit;

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
    void afterReplot();
    void selectionChanged();
    void savePDF();
    void savedata();
    void setup_plot();
    void populate_listwidget();
    void selectedFieldsChanged(int state);
    void selectedFileChanged();
    void lower_bound_changed(double);
    void upper_bound_changed(double);

private:
    void setup_layout();
    void update_spin_boxes();
    void update_time_series_plot();
    void update_fft_plot();
    QCustomPlot * time_plot_;
    QCustomPlot * fft_plot_;
    QList<QString> data_filenames_;
    // Map filename to vector of protobuf Sample messages
    QMap<QString, QVector<sample::Sample>> proto_messages_;
    // Map filename and fieldname to QVector of doubles (time series data)
    // fieldname is roughly the protobuf message field name, it also is used in
    // the main gui for checkboxes
    QMap<QString, QMap<QString, QVector<double>>> time_series_;
    QMap<QString, QMap<QString, gui::MetaData>> time_series_meta_data_;

    QListWidget * listwidget_;
    QFuture<QString> future_;
    QFutureWatcher<QString> watcher_;

    QLineEdit * width_edit_, * height_edit_;
    QDoubleSpinBox * t_lower_spin_box_, * t_upper_spin_box_;

    QVector<QString> fields_;
    QString selected_file_;

    QMap<QString, QCPGraph *> time_graph_map_;

    gui::DataWrangler dw_;
};

} // namespace gui

#endif // MAINWINDOW_H

