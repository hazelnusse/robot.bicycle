// ============================================================================
// 
//         Author:  Dale Lukas Peterson (dlp), hazelnusse@gmail.com
// 
//    Description:  Implementation of QMainWindow subclass
// 
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fftw3.h>

#include <QDebug>
#include <QFile>
#include <QTextStream>

#include <QColor>

#include <QtConcurrentFilter>
#include <QFutureSynchronizer>
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
#include <QTabWidget>

#include "mainwindow.h"
#include "../firmware/src/constants.h"

namespace gui {

const int minwidth = 400;
const int minheight = 400;
const int maxwidth = 1920;
const int maxheight = 1200;
MainWindow::MainWindow(const QVector<QString> & data_filenames, QWidget *parent) :
    QMainWindow(parent),
    fields_{"time", "acc_x", "acc_y", "acc_z",
            "gyro_x", "gyro_y", "gyro_z", "temp",
            "rear_wheel", "rear_wheel_rate", "steer",
            "steer_rate", "T_rw",
            "T_rw_desired", "T_s", "T_s_desired",
            "v", "v_c", "yr_c", "theta_r_dot_lb",
            "theta_r_dot_ub", "lean_est", "steer_est",
            "lean_rate_est", "steer_rate_est", "yaw_rate_est", "hw_button"},
    dw_{proto_messages_, time_series_, time_series_meta_data_, fields_},
    fft_outdated_{true}
{
    setup_pen();
    setup_layout();
    setup_time_plot();
    setup_fft_plot();
    
    // Shortcuts
    new QShortcut(Qt::CTRL + Qt::Key_Q, this, SLOT(close()));
    new QShortcut(Qt::CTRL + Qt::Key_S, this, SLOT(savePDF()));

    QFuture<QString> future_ = QtConcurrent::filtered(data_filenames, dw_);
    QFutureSynchronizer<QString> synchronizer;
    synchronizer.addFuture(future_);
    synchronizer.waitForFinished();
    data_filenames_ = future_.results();

    populate_listwidget();
}

MainWindow::~MainWindow()
{
}

void MainWindow::graphClicked(QCPAbstractPlottable *plottable)
{
    statusBar()->showMessage(QString("Graph '%1' selected.").arg(plottable->name()));
}

void MainWindow::mousePress()
{
  // if an axis is selected, only allow the direction of that axis to be dragged
  // if no axis is selected, both directions may be dragged
  QCustomPlot * plot = qobject_cast<QCustomPlot *>(sender());
  
  if (plot->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
    plot->axisRect()->setRangeDrag(plot->xAxis->orientation());
  else if (plot->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
    plot->axisRect()->setRangeDrag(plot->yAxis->orientation());
  else
    plot->axisRect()->setRangeDrag(Qt::Horizontal|Qt::Vertical);
}

void MainWindow::mouseWheel()
{
  // if an axis is selected, only allow the direction of that axis to be zoomed
  // if no axis is selected, both directions may be zoomed
  QCustomPlot * plot = qobject_cast<QCustomPlot *>(sender());
  
  if (plot->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
    plot->axisRect()->setRangeZoom(plot->xAxis->orientation());
  else if (plot->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
    plot->axisRect()->setRangeZoom(plot->yAxis->orientation());
  else
    plot->axisRect()->setRangeZoom(Qt::Horizontal|Qt::Vertical);
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

  QCustomPlot * plot = qobject_cast<QCustomPlot *>(sender());
  
  // make top and bottom axes be selected synchronously, and handle axis and
  // tick labels as one selectable object:
  if (plot->xAxis->selectedParts().testFlag(QCPAxis::spAxis) || plot->xAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
      plot->xAxis2->selectedParts().testFlag(QCPAxis::spAxis) || plot->xAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
  {
    plot->xAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    plot->xAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }
  // make left and right axes be selected synchronously, and handle axis and tick labels as one selectable object:
  if (plot->yAxis->selectedParts().testFlag(QCPAxis::spAxis) || plot->yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
      plot->yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || plot->yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
  {
    plot->yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
    plot->yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
  }
  
  // synchronize selection of graphs with selection of corresponding legend items:
  for (int i = 0; i < plot->graphCount(); ++i)
  {
    QCPGraph *graph = plot->graph(i);
    QCPPlottableLegendItem *item = plot->legend->itemWithPlottable(graph);
    if (item->selected() || graph->selected())
    {
      item->setSelected(true);
      graph->setSelected(true);
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
    time_plot_ = new QCustomPlot;
    time_plot_->setSizePolicy(qp);
    fft_plot_ = new QCustomPlot;
    fft_plot_->setSizePolicy(qp);
    QTabWidget * tw_ = new QTabWidget;
    tab_indices[0] = tw_->addTab(time_plot_, "&Time domain");
    tab_indices[1] = tw_->addTab(fft_plot_, "&Frequency domain");
    connect(tw_, SIGNAL(currentChanged(int)),
            this, SLOT(tab_changed(int)));
    vr->addWidget(tw_);

    // Strip below plot
    QHBoxLayout * hr = new QHBoxLayout;
    QPushButton * savebutton = new QPushButton("&Save PDF");
    connect(savebutton, SIGNAL(clicked()), this, SLOT(savePDF()));
    hr->addWidget(savebutton);
    QPushButton * reset_horizontal_button = new QPushButton("Reset &horizontal axis");
    connect(reset_horizontal_button, SIGNAL(clicked()), this, SLOT(reset_horizontal_axis()));
    hr->addWidget(reset_horizontal_button);
    QPushButton * reset_vertical_button = new QPushButton("Reset &vertical axis");
    connect(reset_vertical_button, SIGNAL(clicked()), this, SLOT(reset_vertical_axis()));
    hr->addWidget(reset_vertical_button);

    t_lower_spin_box_ = new QDoubleSpinBox;
    t_lower_spin_box_->setSuffix(" s");
    t_lower_spin_box_->setSingleStep(0.25);
    hr->addWidget(t_lower_spin_box_);
    t_upper_spin_box_ = new QDoubleSpinBox;
    t_upper_spin_box_->setSuffix(" s");
    t_upper_spin_box_->setSingleStep(0.25);
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
    time_plot_->savePdf("test.pdf", true, 800, 800);
//                   width_edit_->text().toInt(), height_edit_->text().toInt());
}

void MainWindow::setup_time_plot()
{
    time_plot_->setMinimumSize(minwidth, minheight);
    time_plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                QCP::iSelectLegend | QCP::iSelectPlottables);
    time_plot_->axisRect()->setupFullAxesBox();

    time_plot_->xAxis->setLabel("Time (s)");
    time_plot_->legend->setVisible(true);
    QFont legendFont = font();
    legendFont.setPointSize(10);
    time_plot_->legend->setFont(legendFont);
    time_plot_->legend->setSelectedFont(legendFont);
    // box shall not be selectable, only legend items
    time_plot_->legend->setSelectableParts(QCPLegend::spItems);

    // Connect SIGNALS and SLOTS
    connect(time_plot_, SIGNAL(selectionChangedByUser()),
            this, SLOT(selectionChanged()));
    connect(time_plot_, SIGNAL(mousePress(QMouseEvent*)),
            this, SLOT(mousePress()));
    connect(time_plot_, SIGNAL(mouseWheel(QWheelEvent*)),
            this, SLOT(mouseWheel()));
    connect(time_plot_, SIGNAL(afterReplot()),
            this, SLOT(afterReplot()));
    // connect slot that shows a message in the status bar when a graph is clicked:
    connect(time_plot_, SIGNAL(plottableClick(QCPAbstractPlottable *, QMouseEvent *)),
            this, SLOT(graphClicked(QCPAbstractPlottable *)));
}

void MainWindow::setup_fft_plot()
{
    fft_plot_->setMinimumSize(minwidth, minheight);
    fft_plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                QCP::iSelectLegend | QCP::iSelectPlottables);
    fft_plot_->axisRect()->setupFullAxesBox();

    fft_plot_->xAxis->setLabel("Frequency (Hz)");
    fft_plot_->legend->setVisible(true);
    QFont legendFont = font();
    legendFont.setPointSize(10);
    fft_plot_->legend->setFont(legendFont);
    fft_plot_->legend->setSelectedFont(legendFont);
    // box shall not be selectable, only legend items
    fft_plot_->legend->setSelectableParts(QCPLegend::spItems);


    // Connect SIGNALS and SLOTS
    connect(fft_plot_, SIGNAL(selectionChangedByUser()),
            this, SLOT(selectionChanged()));
    connect(fft_plot_, SIGNAL(mousePress(QMouseEvent*)),
            this, SLOT(mousePress()));
    connect(fft_plot_, SIGNAL(mouseWheel(QWheelEvent*)),
            this, SLOT(mouseWheel()));
    // connect slot that shows a message in the status bar when a graph is clicked:
    connect(fft_plot_, SIGNAL(plottableClick(QCPAbstractPlottable *, QMouseEvent *)),
            this, SLOT(graphClicked(QCPAbstractPlottable *)));
}

void MainWindow::populate_listwidget()
{
    if (data_filenames_.size()) {
        listwidget_->setCurrentItem(new QListWidgetItem(data_filenames_[0], listwidget_));
        selected_file_ = listwidget_->currentItem()->text();
        i_lower_ = 0;
        i_upper_ = time_series_[selected_file_]["time"].size() - 1;
        length_ = time_series_[selected_file_]["time"].size();
    }

    for (int i = 1; i < data_filenames_.size(); ++i)
        new QListWidgetItem(data_filenames_[i], listwidget_);

    connect(listwidget_, SIGNAL(itemSelectionChanged()),
            this, SLOT(selectedFileChanged()));

    connect(t_lower_spin_box_, SIGNAL(valueChanged(double)),
            this, SLOT(lower_bound_changed(double)));
    connect(t_upper_spin_box_, SIGNAL(valueChanged(double)),
            this, SLOT(upper_bound_changed(double)));

    selectedFileChanged();
    statusBar()->showMessage(tr("Ready"));
}

void MainWindow::selectedFileChanged()
{
    selected_file_ = listwidget_->currentItem()->text();
    time_plot_->clearGraphs();

    for (const QString & field : time_graph_map_.keys()) {
        QCPGraph * graph = time_plot_->addGraph();
        graph->setData(time_series_[selected_file_]["time"],
                                time_series_[selected_file_][field]);
        graph->setName(field);
        graph->setPen(pen[field]);
        time_graph_map_[field] = graph;
    }

    const double t_min = time_series_[selected_file_]["time"].first();
    const double t_max = time_series_[selected_file_]["time"].last();
    time_plot_->xAxis->setRange(t_min, t_max);
    update_spin_boxes();
    update_time_series_plot();
    fft_outdated_ = true;
    update_fft_plot();
}

void MainWindow::selectedFieldsChanged(int state)
{
    QCheckBox * box = qobject_cast<QCheckBox *>(sender());
    QString field = box->text();

    if (state == Qt::Checked) {
        QCPGraph * graph = time_plot_->addGraph();
        graph->setData(time_series_[selected_file_]["time"],
                                time_series_[selected_file_][field]);
        graph->setName(field);
        graph->setPen(pen[field]);
        time_graph_map_.insert(box->text(), graph);
    } else if (state == Qt::Unchecked) {
        time_plot_->removeGraph(time_graph_map_[field]);
        time_graph_map_.remove(field);
        fft_plot_->removeGraph(fft_graph_map_[field]);
        fft_graph_map_.remove(field);
    }

    update_time_series_plot();
    fft_outdated_ = false;  // Indicate only the added field needs the FFT computed.
    update_fft_plot();
}

void MainWindow::lower_bound_changed(double bound)
{
    t_upper_spin_box_->setRange(bound, *(time_series_[selected_file_]["time"].end() - 1));

    time_plot_->xAxis->setRangeLower(bound);
    time_plot_->replot();
}

void MainWindow::upper_bound_changed(double bound)
{
    t_lower_spin_box_->setRange(0.0, bound);

    time_plot_->xAxis->setRangeUpper(bound);
    time_plot_->replot();
}

void MainWindow::savedata()
{
    const QVector<double> & t = time_series_[selected_file_]["time"];
    const double lb = t[i_lower_];
    const double ub = t[i_upper_ - 1];
    QString suggested_dir_base = "/home/luke/repos/dissertation/data/";

    QString file_contents = "time ";
    QString file_contents_fft = "freq ";
    for (auto field : time_graph_map_.keys()) {
        file_contents += field + " ";
        file_contents_fft += field + " ";
    }
    file_contents.chop(1); // Remove trailing space
    file_contents_fft.chop(1);
    QString description_mid = file_contents;
    QString description_mid_fft = file_contents_fft;
    description_mid.replace(QString(" "), QString("-"));
    description_mid_fft.replace(QString(" "), QString("-"));
    
    file_contents.prepend("# Generated from " + selected_file_ + "\n");
    file_contents += "\n"; // Add newline
    file_contents_fft.prepend("# Generated from " + selected_file_ + "\n");
    file_contents_fft += "\n"; // Add newline

    QString file_contents_decimated = file_contents;
    QString file_contents_decimated_fft = file_contents_fft;

    QString suggested_filename = selected_file_.split("/").last().split(".").first() + "-";
    QString suggested_filename_fft = selected_file_.split("/").last().split(".").first() + "-";

    suggested_filename += description_mid + "-";
    suggested_filename += QString::number(lb, 'f', 2) + "-" + QString::number(ub, 'f', 2) + ".txt";
    suggested_filename_fft += description_mid_fft + "-";
    suggested_filename_fft += QString::number(lb, 'f', 2) + "-" + QString::number(ub, 'f', 2) + "-fft.txt";

    QString filename = QFileDialog::getSaveFileName(this, tr("Save File"),
            suggested_dir_base + suggested_filename, 
            tr("Data files (*.txt)"));
    if (filename == "")
        return;
    QString filename_decimated = filename;
    filename_decimated.insert(filename.size() - 4, "-decimated");

    for (int i = i_lower_; i < i_upper_; ++i) {
        QString time = QString::number(time_series_[selected_file_]["time"][i]) + " ";
        file_contents += time;
        if (i % 4 == 0)
            file_contents_decimated += time;

        for (auto field : time_graph_map_.keys()) {
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

    write_file(filename, file_contents);
    write_file(filename_decimated, file_contents_decimated);

    // Get FFT filename
    QString filename_fft = QFileDialog::getSaveFileName(this, tr("Save FFT File"),
            suggested_dir_base + suggested_filename_fft, 
            tr("Data files (*.txt)"));
    QString filename_decimated_fft = filename_fft;
    filename_decimated_fft.insert(filename_fft.size() - 4, "_decimated");
    if (filename_fft == "")
        return;

    // Get FFT data in string form
    for (int i = 0; i < fft_freqs_.size(); ++i) {
        QString freq = QString::number(fft_freqs_[i]) + " ";
        file_contents_fft += freq;
        if (i % 4 == 0)
            file_contents_decimated_fft += freq;

        for (auto field : fft_data_mag_.keys()) {
            QString sample = QString::number(fft_data_mag_[field][i]) + " ";
            file_contents_fft += sample;
            if (i % 4 == 0) {
                file_contents_decimated_fft += sample;
            }
        }

        if (i % 4 == 0) {
            file_contents_decimated_fft.chop(1);
            file_contents_decimated_fft += "\n";
        }
        file_contents_fft.chop(1);
        file_contents_fft += "\n";
    }
    write_file(filename_fft, file_contents_fft);
    write_file(filename_decimated_fft, file_contents_decimated_fft);
}

void MainWindow::update_spin_boxes()
{
    QCPRange range = time_plot_->xAxis->range();
    t_lower_spin_box_->setValue(range.lower);
    t_upper_spin_box_->setValue(range.upper);

    const QVector<double> & t = time_series_[selected_file_]["time"];
    // lower_bound returns the *first* element that is >= range.lower
    const double *lb = std::lower_bound(t.begin(), t.end(), range.lower);
    if (lb != t.begin()) // Include the point just below
        lb -= 1;         // Also fixes the case when lb==t.end()
    // upper_bound returns the *first* element that is > range.upper
    const double *ub = std::upper_bound(t.begin(), t.end(), range.upper);
    if (ub == t.end())   // Fixes case when up==t.end()
        ub -= 1;      
    // at this point, ub and lb can be safely dereferenced to point to elements
    // inside the time array.
    i_lower_ = t.indexOf(*lb);
    i_upper_ = t.indexOf(*ub);
    length_ = i_upper_ - i_lower_ + 1;
    fft_outdated_ = true;
}

void MainWindow::afterReplot()
{
    update_spin_boxes();
}

void MainWindow::update_time_series_plot()
{
    update_spin_boxes();

    double y_min = std::numeric_limits<double>::max(),
           y_max = std::numeric_limits<double>::min();

    for (const auto & s : time_graph_map_.keys()) {
        QVector<double> signal = time_series_[selected_file_][s].mid(i_lower_, length_);
        gui::MetaData md = gui::MetaData(signal);
        y_min = std::min(y_min, md.min_);
        y_max = std::max(y_max, md.max_);
    }
    time_plot_->yAxis->setRange(y_min, y_max);
    time_plot_->replot();
}

void MainWindow::update_fft_plot()
{
    fft_plot_->clearGraphs();       // Clear plots
    fft_data_.clear();              // Clear FFT data
    fft_data_mag_.clear();          // Clear FFT data
    fft_data_mag_.clear();          // Clear FFT data

    const QMap<QString, QVector<double>> & data = time_series_[selected_file_];
    int N_real = length_;
    int N_complex = N_real / 2 + 1;
     // save plans until we have computed all fft's, this makes fftw use
     // "wisdom" and should help keep things fast
    QVector<fftw_plan> plans;
    fft_freqs_.clear(); fft_freqs_.reserve(N_complex);
    double f_sample = 1.0 / constants::loop_period_s;
    for (int i = 0; i < N_complex; ++i)
        fft_freqs_.push_back(i * f_sample / N_real);

    double * in = static_cast<double *>(
            fftw_malloc(sizeof(double) * N_real));
    std::complex<double> * out = static_cast<std::complex<double> *>(
            fftw_malloc(sizeof(std::complex<double>) * N_complex));

    fft_mag_max_ = std::numeric_limits<double>::min();
    for (const QString & field : time_graph_map_.keys()) {
        // Copy the data to the input vector and compute the mean
        double mean = 0.0;
        const double * data_field_lower_bound = data[field].data() + i_lower_;
        for (int i = 0; i < N_real; ++i) {
            in[i] = data_field_lower_bound[i];
            mean += in[i];
        }
        mean /= N_real;

        for (int i = 0; i < N_real; ++i)
            in[i] -= mean;

        // Build an FFT Plan
        fftw_plan p = fftw_plan_dft_r2c_1d(N_real,
                                           in,
                                           reinterpret_cast<fftw_complex *>(out),
                                           FFTW_ESTIMATE | FFTW_DESTROY_INPUT);

        // Execute the plan
        fftw_execute(p);
        plans.push_back(p);

        QVector<std::complex<double>> out_vec;
        out_vec.reserve(N_complex); out_vec.resize(N_complex);
        std::memcpy(out_vec.data(), out, N_complex * sizeof(std::complex<double>));
        fft_data_.insert(field, out_vec);
        QVector<double> out_vec_mag;
        QVector<double> out_vec_phase;
        out_vec_mag.reserve(N_complex); out_vec_mag.resize(N_complex);
        out_vec_phase.reserve(N_complex); out_vec_phase.resize(N_complex);
        for (int i = 0; i < N_complex; ++i) {
            out_vec_mag[i] = std::abs(out_vec[i]) / N_complex;
            out_vec_phase[i] = std::atan2(out_vec[i].imag(), out_vec[i].real()) / N_complex;
            if (out_vec_mag[i] > fft_mag_max_)
                fft_mag_max_ = out_vec_mag[i];
        }
        fft_data_mag_.insert(field, out_vec_mag);
        fft_data_phase_.insert(field, out_vec_phase);

        QCPGraph * graph = fft_plot_->addGraph();
        graph->setData(fft_freqs_, out_vec_mag);
        graph->setName(field);
        graph->setPen(pen[field]);
        fft_graph_map_[field] = graph;
    }
    for (int i = 0; i < plans.size(); ++i) fftw_destroy_plan(plans[i]);
    fftw_free(in); fftw_free(reinterpret_cast<fftw_complex *>(out));

    fft_plot_->xAxis->setRange(fft_freqs_.first(), fft_freqs_.last());
    fft_plot_->yAxis->setRange(0, fft_mag_max_);
    fft_plot_->replot();
    fft_outdated_ = false;
}

void MainWindow::reset_horizontal_axis()
{
    const double t_min = *time_series_[selected_file_]["time"].begin();
    const double t_max = *(time_series_[selected_file_]["time"].end() - 1);
    time_plot_->xAxis->setRange(t_min, t_max);
    time_plot_->replot();
    fft_plot_->xAxis->setRange(0, 1.0 / (2.0 * constants::loop_period_s));
    fft_plot_->replot();
}

void MainWindow::reset_vertical_axis()
{
    update_time_series_plot();

    update_fft_vertical_range();
}

void MainWindow::tab_changed(int index)
{
    if (index == tab_indices[1] && fft_outdated_) { // FFT Plot
        update_fft_plot();
    }
}

void MainWindow::setup_pen()
{
    for (const auto & field : fields_) {
        QPen graphPen;
        graphPen.setColor(QColor(rand()%245+10, rand()%245+10, rand()%245+10));
        pen.insert(field, graphPen);
    }

}

void MainWindow::write_file(const QString & filename, const QString & file_contents)
{
    QFile file(filename);
    file.open(QIODevice::WriteOnly);
    QTextStream out(&file);
    out << file_contents;
}

void MainWindow::update_fft_vertical_range()
{
    QCPRange range = fft_plot_->xAxis->range();
    double f_min = range.lower,
           f_max = range.upper;

    const QVector<double> & f = fft_freqs_;
    // lower_bound returns the *first* element that is >= range.lower
    const double *lb = std::lower_bound(f.begin(), f.end(), f_min);
    if (lb != f.begin()) // Include the point just below
        lb -= 1;         // Also fixes the case when lb==t.end()
    // upper_bound returns the *first* element that is > range.upper
    const double *ub = std::upper_bound(f.begin(), f.end(), f_max);
    if (ub == f.end())   // Fixes case when up==t.end()
        ub -= 1;      
    // at this point, ub and lb can be safely dereferenced to point to elements
    // inside the frequency array.
    int fft_i_lower = f.indexOf(*lb);
    int fft_i_upper = f.indexOf(*ub);
    int length = fft_i_upper - fft_i_lower + 1;
    // Now find maximum frequency over the range we are looking at
    double y_min = std::numeric_limits<double>::max(),
           y_max = std::numeric_limits<double>::min();

    for (const auto & s : fft_graph_map_.keys()) {
        const QVector<double> signal = fft_data_mag_[s].mid(fft_i_lower, length);
        gui::MetaData md = gui::MetaData(signal);
        y_min = std::min(y_min, md.min_);
        y_max = std::max(y_max, md.max_);
    }
    fft_plot_->yAxis->setRange(y_min, y_max);
    fft_plot_->replot();
}



} // namespace gui

