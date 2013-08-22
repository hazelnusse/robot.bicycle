// ============================================================================
// 
//         Author:  Dale Lukas Peterson (dlp), hazelnusse@gmail.com
// 
//    Description:  Implementation of QMainWindow subclass
// 
// ============================================================================

#include <QDebug>

#include <QFuture>
#include <QtConcurrentFilter>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <QCheckBox>
#include <QStatusBar>
#include <QInputDialog>
#include <QPushButton>
#include <QShortcut>

#include <QIntValidator>

#include "mainwindow.h"
#include "sample.pb.h"

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
    dw_{proto_messages_, time_series_, fields_}
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
    plot_->plotLayout()->insertRow(0);
    plot_->plotLayout()->addElement(0, 0, new QCPPlotTitle(plot_, "Interaction Example"));

    plot_->xAxis->setLabel("Time (s)");
    plot_->yAxis->setLabel("y Axis");
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
}

void MainWindow::selectedFileChanged()
{
    selected_file_ = listwidget_->currentItem()->text();
    // need to update plot
    plot_->clearGraphs();

    QMap<QString, QCPGraph *>::iterator i;
    // for (i = selected_fields_.begin(); i != selected_fields_.end(); ++i) {
    for (i = selected_fields_.begin(); i != selected_fields_.end(); ++i) {
        QCPGraph * graph = plot_->addGraph();
        plot_->graph()->setData(time_series_[selected_file_]["time"],
                                time_series_[selected_file_][i.key()]);
        plot_->graph()->setName(i.key());
        selected_fields_[i.key()] = graph;
    }
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
    plot_->replot();
}

//void MainWindow::update_plot()
//{
//
////    QVector<double> x(101), y(101); // initialize with entries 0..100
////    for (int i = 0; i < 101; ++i) {
////      x[i] = i/50.0 - 1; // x goes from -1 to 1
////      y[i] = x[i]*x[i];  // let's plot a quadratic function
////    }
////    plot_->addGraph();
////    plot_->graph(0)->setData(x, y);
////    plot_->graph(0)->setName("y = x^2");
//    // plot_->xAxis->setRange(-1, 1);
//    // plot_->yAxis->setRange(0, 1);
//}

} // namespace gui

