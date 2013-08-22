#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include <QMap>
#include <QMutex>
#include <QString>
#include <QVector>
#include "sample.pb.h"

namespace gui {

class DataWrangler {
public:
    DataWrangler(QMap<QString, QVector<sample::Sample>> & proto_messages,
                 QMap<QString, QMap<QString, QVector<double>>> & time_series,
                 const QVector<QString> & fieldnames);
    ~DataWrangler();

    bool operator()(QString const & filename);
private:
    QMap<QString, QVector<double>> to_time_series(const QVector<sample::Sample> & sv);

    QMap<QString, QVector<sample::Sample>> & proto_messages_;
    QMap<QString, QMap<QString, QVector<double>>> & time_series_;
    const QVector<QString> & fieldnames_;
    static QMutex mutex_;
};

} // namesapce gui

#endif

