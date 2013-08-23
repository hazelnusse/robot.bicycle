#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include <QMap>
#include <QMutex>
#include <QString>
#include <QVector>
#include "sample.pb.h"
#include "run_meta_data.h"

namespace gui {

class DataWrangler {
public:
    DataWrangler(QMap<QString, QVector<sample::Sample>> & proto_messages,
                 QMap<QString, QMap<QString, QVector<double>>> & time_series,
                 QMap<QString, QMap<QString, gui::MetaData>> & time_series_meta_data,
                 const QVector<QString> & fieldnames);
    ~DataWrangler();

    bool operator()(QString const & filename);
private:
    QMap<QString, QVector<double>> to_time_series(const QVector<sample::Sample> & sv);
    QMap<QString, gui::MetaData> compute_meta_data(const QMap<QString, QVector<double>> & ts_data);

    QMap<QString, QVector<sample::Sample>> & proto_messages_;
    QMap<QString, QMap<QString, QVector<double>>> & time_series_;
    QMap<QString, QMap<QString, gui::MetaData>> & time_series_meta_data_;
    const QVector<QString> & fieldnames_;
    static QMutex mutex_;
};

} // namesapce gui

#endif

