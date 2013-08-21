#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include <QMap>
#include <QMutex>
#include <QString>
#include <QVector>
#include "sample.pb.h"

namespace gui {

class DataLoader {
public:
    DataLoader(QMap<QString, QVector<sample::Sample>> & data_set,
               QVector<QString> filenames);
    ~DataLoader();
    
    bool operator()(QString const & filename);
private:
    QMap<QString, QVector<sample::Sample>> & data_set_;
    QVector<QString> filenames_;
    static QMutex mutex_;
};

} // namesapce gui

#endif

