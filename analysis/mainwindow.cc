// ============================================================================
// 
//         Author:  Dale Lukas Peterson (dlp), hazelnusse@gmail.com
// 
//    Description:  Implementation of QMainWindow subclass
// 
// ============================================================================

#include <algorithm>

#include <QFile>
#include <QTextStream>

#include <QtConcurrentFilter>
#include <QShortcut>
#include <QIntValidator>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QStatusBar>

#include "mainwindow.h"

namespace gui {

const int minwidth = 400;
const int minheight = 400;
const int maxwidth = 1920;
const int maxheight = 1200;
MainWindow::MainWindow(const QVector<QString> & data_filenames, QWidget *parent) :
    QMainWindow(parent),
    plot_{new QCustomPlot(this)},
    fields_{"time", "acc_x", "acc_y", "acc_z",
            "gyro_x", "gyro_y", "gyro_z", "temp",
            "rear_wheel", "rear_wheel_rate", "steer",
            "steer_rate", "T_rw",
            "T_rw_desired", "T_s", "T_s_desired",
            "v", "v_c", "yr_c", "theta_r_dot_lb",
            "theta_r_dot_ub", "lean_est", "steer_est",
            "lean_rate_est", "steer_rate_est", "yaw_rate_est"},
    dw_{proto_messages_, time_series_, time_series_meta_data_, fields_}
{
    setup_layout();
    setup_plot();
    
    // Shortcuts
    new QShortcut(Qt::CTRL + Qt::Key_Q, this, SLOT(close()));
    new QShortcut(Qt::CTRL + Qt::Key_S, this, SLOT(savePDF()));

    connect(&watcher_, SIGNAL(finished()),
            this, SLOT(populate_listwidget()));
    future_ = QtConcurrent::filtered(data_filenames, dw_);
    watcher_.setFuture(future_);
}

MainWindow::~MainWindow()
{
}

void MainWindow::graphClicked(QCPAbstractPlottable *plottable)
{
    statusBar()->showMessage(QString("Graph '%1' selected.").arg(plottable->name()));
}

void MainWindow::legendDoubleClick(QCPLegend *, QCPAbstractLegendItem *item)
{
  // Rename a graph by double clicking on its legend item

  // only react if item was clicked (user could have clicked on border padding
  // of legend where there is no item, then item is 0)
  if (item) {
      QCPPlottableLegendItem * plItem = qobject_cast<QCPPlottableLegendItem *>(item);
      bool ok;
      QString newName = QInputDialog::getText(this, "QCustomPlot example",
              "New graph name:", QLineEdit::Normal, plItem->plottable()->name(), &ok);
      if (ok) {
        plItem->plottable()->setName(newName);
        plot_->replot();
      }
  }
}

void MainWindow::mousePress()
{
  // if an axis is selected, only allow the direction of that axis to be dragged
  // if no axis is selected, both directions may be dragged
  
  if (plot_->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
    plot_->axisRect()->setRangeDrag(plot_->xAxis->orientation());
  else if (plot_->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
    plot_->axisRect()->setRangeDrag(plot_->yAxis->orientation());
  else
    plot_->axisRect()->setRangeDrag(Qt::Horizontal|Qt::Vertical);

  QCPRange range = plot_->xAxis->range();
  t_lower_spin_box_->setValue(range.lower);
  t_upper_spin_box_->setValue(range.upper);

}

void MainWindow::mouseWheel()
{
  // if an axis is selected, only allow the direction of that axis to be zoomed
  // if no axis is selected, both directions may be zoomed
  
  if (plot_->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
    plot_->axisRect()->setRangeZoom(plot_->xAxis->orientation());
  else if (plot_->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
    plot_->axisRect()->setRangeZoom(plot_->yAxis->orientation());
  else
    plot_->axisRect()->setRangeZoom(Qt::Horizontal|Qt::Vertical);

  QCPRange range = plot_->xAxis->range();
  t_lower_spin_box_->setValue(range.lower);
  t_upper_spin_box_->setValue(range.upper);
}

void MainWindow::selectionChanged()
{
  /*
   normally, axis base line, axis tick labels and axis labels are selectable
   separately, but we want the user only to be able to select the axis as a
   whole, so we tie the selected states of the tick labels and the axis base
   line together. However, the axis label shall be selectable individually.
   
   The selection state of the left and right axes shall be synchronized as well
   as the state of the bottom and top axes.
   
   Further, we want to synchronize the selection of the graphs with the
   selection state of the respective legend item belonging to that graph. So
   the user can select a graph by either clicking on the graph itself or on its
   legend item.
  */
  
  // make top and bottom axes be selected synchronously, and handle axis and
  // tick labels as one selectable object:
  if (plot_->xAxis->selectedParts().testFlag(QCPAxis::spAxis) || plot_->xAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
      plot_->xAxis2->selectedParts().testFlag(QCPAxis::spAxis) || plot_->xAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
  {
    plot_->xAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    plot_->xAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }
  // make left and right axes be selected synchronously, and handle axis and tick labels as one selectable object:
  if (plot_->yAxis->selectedParts().testFlag(QCPAxis::spAxis) || plot_->yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
      plot_->yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || plot_->yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
  {
    plot_->yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    plot_->yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }
  
  // synchronize selection of graphs with selection of corresponding legend items:
  for (int i = 0; i < plot_->graphCount(); ++i)
  {
    QCPGraph *graph = plot_->graph(i);
    QCPPlottableLegendItem *item = plot_->legend->itemWithPlottable(graph);
    if (item->selected() || graph->selected())
    {
      item->setSelected(true);
      graph->setSelected(true);
    }
  }
}

void MainWindow::titleDoubleClick(QMouseEvent *, QCPPlotTitle * title)
{
  // Set the plot title by double clicking on it
  bool ok;
  QString newTitle = QInputDialog::getText(this, "QCustomPlot example",
          "New plot title:", QLineEdit::Normal, title->text(), &ok);
  if (ok) {
    title->setText(newTitle);
    plot_->replot();
  }
}

void MainWindow::axisLabelDoubleClick(QCPAxis *axis, QCPAxis::SelectablePart part)
{
  // Set an axis label by double clicking on it
  // only react when the actual axis label is clicked, not tick label or axis backbone
  if (part == QCPAxis::spAxisLabel) {
    bool ok;
    QString newLabel = QInputDialog::getText(this, "QCustomPlot example",
            "New axis label:", QLineEdit::Normal, axis->label(), &ok);
    if (ok) {
      axis->setLabel(newLabel);
      plot_->replot();
    }
  }
}

void MainWindow::setup_layout()
{
    // Left half of window has a vbox with a list widget and check box grid
    QSizePolicy qp(QSizePolicy::Minimum, QSizePolicy::Preferred);
    listwidget_ = new QListWidget;
    listwidget_->setSizePolicy(qp);
    listwidget_->setSortingEnabled(true);
    QGridLayout * grid = new QGridLayout;
    int cols = 3;
    int rows = (fields_.size() + (cols - 1)) / cols;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (cols * i + j < fields_.size()) {
                QCheckBox * cb = new QCheckBox(fields_[cols * i + j]);
                connect(cb, SIGNAL(stateChanged(int)),
                        this, SLOT(selectedFieldsChanged(int)));
                grid->addWidget(cb, i, j);
            }
        }
    }

    QVBoxLayout * vl = new QVBoxLayout;
    vl->addWidget(listwidget_);
    vl->addLayout(grid);

    // Right half has a vbox with the plot on top and a small row with a button
    // to save the figure, two fields for the figure size, and three radio
    // buttons which permit square, golden, or arbitrary aspect ratios. When
    // square or golden are selected, changing the height or width will auto
    // matically resize the other
    QVBoxLayout * vr = new QVBoxLayout;
    qp.setHorizontalPolicy(QSizePolicy::MinimumExpanding);
    plot_->setSizePolicy(qp);
    plot_->setMinimumSize(minwidth, minheight);
    // plot_->setMaximumSize(maxwidth, maxheight);
    vr->addWidget(plot_);

    // Strip below plot
    QHBoxLayout * hr = new QHBoxLayout;
    QPushButton * savebutton = new QPushButton("&Save PDF");
    connect(savebutton, SIGNAL(clicked()), this, SLOT(savePDF()));
    hr->addWidget(savebutton);

    width_edit_ = new QLineEdit;
    QIntValidator * width_validator = new QIntValidator(minwidth, maxwidth, this);
    width_edit_->setValidator(width_validator);
    width_edit_->setText(QString::number(minwidth));
    hr->addWidget(width_edit_);

    height_edit_ = new QLineEdit;
    QIntValidator * height_validator = new QIntValidator(minheight, maxheight, this);
    height_edit_->setValidator(height_validator);
    height_edit_->setText(QString::number(minheight));
    hr->addWidget(height_edit_);

    t_lower_spin_box_ = new QDoubleSpinBox;
    t_lower_spin_box_->setSuffix(" s");
    hr->addWidget(t_lower_spin_box_);
    t_upper_spin_box_ = new QDoubleSpinBox;
    t_upper_spin_box_->setSuffix(" s");
    hr->addWidget(t_upper_spin_box_);

    QPushButton * savedatabutton = new QPushButton("Save &data");
    connect(savedatabutton, SIGNAL(clicked()), this, SLOT(savedata()));
    hr->addWidget(savedatabutton);

    vr->addLayout(hr);
    
    QHBoxLayout * hl = new QHBoxLayout;
    hl->addLayout(vl);
    hl->addLayout(vr);

    QWidget * widget = new QWidget;
    widget->setLayout(hl);
    setCentralWidget(widget);
}

void MainWindow::savePDF()
{
    plot_->savePdf("test.pdf", true,
                   width_edit_->text().toInt(), height_edit_->text().toInt());
}

void MainWindow::setup_plot()
{
    plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                           QCP::iSelectLegend | QCP::iSelectPlottables);
    plot_->axisRect()->setupFullAxesBox();

    plot_->xAxis->setLabel("Time (s)");
    // plot_->yAxis->setLabel("y Axis");
    plot_->legend->setVisible(true);
    QFont legendFont = font();
    legendFont.setPointSize(10);
    plot_->legend->setFont(legendFont);
    plot_->legend->setSelectedFont(legendFont);
    // box shall not be selectable, only legend items
    plot_->legend->setSelectableParts(QCPLegend::spItems);

    statusBar()->showMessage(tr("Ready"));

    // Connect SIGNALS and SLOTS
    connect(plot_,
            SIGNAL(axisDoubleClick(QCPAxis*,QCPAxis::SelectablePart,QMouseEvent*)),
            this,
            SLOT(axisLabelDoubleClick(QCPAxis*,QCPAxis::SelectablePart)));
    connect(plot_, SIGNAL(titleDoubleClick(QMouseEvent*,QCPPlotTitle*)),
            this, SLOT(titleDoubleClick(QMouseEvent*,QCPPlotTitle*)));
    connect(plot_, SIGNAL(selectionChangedByUser()),
            this, SLOT(selectionChanged()));
    connect(plot_, SIGNAL(mousePress(QMouseEvent*)),
            this, SLOT(mousePress()));
    connect(plot_, SIGNAL(mouseWheel(QWheelEvent*)),
            this, SLOT(mouseWheel()));
    // connect slot that shows a message in the status bar when a graph is clicked:
    connect(plot_, SIGNAL(plottableClick(QCPAbstractPlottable *, QMouseEvent *)),
            this, SLOT(graphClicked(QCPAbstractPlottable *)));
    connect(plot_,
            SIGNAL(legendDoubleClick(QCPLegend *, QCPAbstractLegendItem *, QMouseEvent *)),
            this,
            SLOT(legendDoubleClick(QCPLegend *, QCPAbstractLegendItem *)));
}

void MainWindow::populate_listwidget()
{
    data_filenames_ = future_.results();

    if (data_filenames_.size()) {
        listwidget_->setCurrentItem(new QListWidgetItem(data_filenames_[0], listwidget_));
        selected_file_ = data_filenames_[0];
    }

    for (int i = 1; i < data_filenames_.size(); ++i)
        new QListWidgetItem(data_filenames_[i], listwidget_);

    connect(listwidget_, SIGNAL(itemSelectionChanged()),
            this, SLOT(selectedFileChanged()));

    // Also configure the spin boxes for the time range
    const double t_min = *time_series_[selected_file_]["time"].begin();
    const double t_max = *(time_series_[selected_file_]["time"].end() - 1);
    t_lower_spin_box_->setRange(t_min, t_max);
    t_upper_spin_box_->setRange(t_min, t_max);
    t_lower_spin_box_->setSingleStep(0.25);
    t_upper_spin_box_->setSingleStep(0.25);
    t_lower_spin_box_->setValue(t_min);
    t_upper_spin_box_->setValue(t_max);

    connect(t_lower_spin_box_, SIGNAL(valueChanged(double)),
            this, SLOT(lower_bound_changed(double)));
    connect(t_upper_spin_box_, SIGNAL(valueChanged(double)),
            this, SLOT(upper_bound_changed(double)));

   plot_->xAxis->setRange(t_min, t_max);
}

void MainWindow::selectedFileChanged()
{
    selected_file_ = listwidget_->currentItem()->text();
    plot_->clearGraphs();

    for (const QString & field : selected_fields_.keys()) {
        QCPGraph * graph = plot_->addGraph();
        plot_->graph()->setData(time_series_[selected_file_]["time"],
                                time_series_[selected_file_][field]);
        plot_->graph()->setName(field);
        selected_fields_[field] = graph;
    }

    const double t_min = *time_series_[selected_file_]["time"].begin();
    const double t_max = *(time_series_[selected_file_]["time"].end() - 1);
    plot_->xAxis->setRange(t_min, t_max);

    double y_min = std::numeric_limits<double>::max(),
           y_max = std::numeric_limits<double>::min();
    const QMap<QString, gui::MetaData> & meta_data = time_series_meta_data_[selected_file_];
    for (const auto & s : selected_fields_.keys()) {
         y_min = std::min(y_min, meta_data[s].min_);
         y_max = std::max(y_max, meta_data[s].max_);
    }
    plot_->yAxis->setRange(y_min, y_max);

    t_lower_spin_box_->setValue(t_min);
    t_upper_spin_box_->setValue(t_max);

    plot_->replot();
}

void MainWindow::selectedFieldsChanged(int state)
{
    QCheckBox * box = qobject_cast<QCheckBox *>(sender());
    QString signal_name = box->text();

    if (state == Qt::Checked) {
        QCPGraph * graph = plot_->addGraph();
        plot_->graph()->setData(time_series_[selected_file_]["time"],
                                time_series_[selected_file_][signal_name]);
        plot_->graph()->setName(signal_name);
        selected_fields_.insert(box->text(), graph);
    } else if (state == Qt::Unchecked) {
        QCPGraph * graph = selected_fields_[signal_name];
        plot_->removeGraph(graph);
        selected_fields_.remove(signal_name);
    }

    double y_min = std::numeric_limits<double>::max(),
           y_max = std::numeric_limits<double>::min();
    const QMap<QString, gui::MetaData> & meta_data = time_series_meta_data_[selected_file_];
    for (const auto & s : selected_fields_.keys()) {
        y_min = std::min(y_min, meta_data[s].min_);
        y_max = std::max(y_max, meta_data[s].max_);
    }
    plot_->yAxis->setRange(y_min, y_max);
    plot_->replot();
}

void MainWindow::lower_bound_changed(double bound)
{
    t_upper_spin_box_->setRange(bound, *(time_series_[selected_file_]["time"].end() - 1));

    plot_->xAxis->setRangeLower(bound);
    plot_->replot();
}

void MainWindow::upper_bound_changed(double bound)
{
    t_lower_spin_box_->setRange(0.0, bound);

    plot_->xAxis->setRangeUpper(bound);
    plot_->replot();
}

void MainWindow::savedata()
{
    const QVector<double> & t = time_series_[selected_file_]["time"];
    const double *lb = std::lower_bound(t.begin(), t.end(), t_lower_spin_box_->value());
    if (lb != t.begin())
        lb -= 1;
    const double *ub = std::upper_bound(t.begin(), t.end(), t_upper_spin_box_->value());

    QString file_contents = "time ";
    for (auto field : selected_fields_.keys())
        file_contents += field + " ";
    file_contents.chop(1); // Remove trailing space
    QString description_mid = file_contents;
    description_mid.replace(QString(" "), QString("_"));
    
    file_contents.prepend("# Generated from " + selected_file_ + "\n");
    file_contents += "\n"; // Add newline
    QString file_contents_decimated = file_contents;

    QString suggested_filename = selected_file_.split("/").last().split(".").first() + "_";
    suggested_filename += description_mid + "_";
    suggested_filename += QString::number(*lb, 'f', 3) + "_" + QString::number(*ub, 'f', 3) + ".dat";

    QString filename = QFileDialog::getSaveFileName(this, tr("Save File"),
            "/home/luke/repos/dissertation/images/" + suggested_filename, 
            tr("Data files (*.dat)"));
    QString filename_decimated = filename;
    filename_decimated.insert(filename.size() - 4, "_decimated");

    for (int i = lb - t.begin(); i < ub - t.begin(); ++i) {
        QString time = QString::number(time_series_[selected_file_]["time"][i]) + " ";
        file_contents += time;
        if (i % 4 == 0)
            file_contents_decimated += time;

        for (auto field : selected_fields_.keys()) {
            QString sample = QString::number(time_series_[selected_file_][field][i]) + " ";
            file_contents += sample;
            if (i % 4 == 0) {
                file_contents_decimated += sample;
            }
        }

        if (i % 4 == 0) {
            file_contents_decimated.chop(1);
            file_contents_decimated += "\n";
        }
        file_contents.chop(1);
        file_contents += "\n";
    }

    {
        QFile file(filename);
        file.open(QIODevice::WriteOnly);
        QTextStream out(&file);
        out << file_contents;
    }
    {
        QFile file(filename_decimated);
        file.open(QIODevice::WriteOnly);
        QTextStream out(&file);
        out << file_contents_decimated;
    }
    
}


} // namespace gui

