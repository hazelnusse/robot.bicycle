#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <iostream>

#include "sample.pb.h"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include "datawrangler.h"
#include "../firmware/src/constants.h"

namespace gui {

// Have to define the mutex
QMutex DataWrangler::mutex_;

DataWrangler::DataWrangler(QMap<QString, QVector<sample::Sample>> & proto_messages,
                           QMap<QString, QMap<QString, QVector<double>>> & time_series,
                           QMap<QString, QMap<QString, gui::MetaData>> & time_series_meta_data,
                           const QVector<QString> & fieldnames) :
    proto_messages_(proto_messages),
    time_series_(time_series),
    time_series_meta_data_(time_series_meta_data),
    fieldnames_(fieldnames)
{

}

bool DataWrangler::operator()(QString const & filename)
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    using google::protobuf::io::ZeroCopyInputStream;
    using google::protobuf::io::FileInputStream;
    using google::protobuf::io::CodedInputStream;

    uint8_t ar[2]; uint16_t bytes;
    QVector<sample::Sample> sv; sv.reserve(200*120);
    sample::Sample s;

    int fd = open(filename.toLocal8Bit(), O_RDONLY);
    if (fd < 0) {
        std::cerr << "Error opening file " 
                  << filename.toLocal8Bit().constData() << std::endl;
        return false;
    }

    FileInputStream file_input(fd);
    CodedInputStream coded_input(&file_input);

    while (coded_input.ReadRaw(ar, 2)) { // Messages are separated by two bytes
        bytes = (ar[1] << 8) | ar[0];
        CodedInputStream::Limit old_limit = coded_input.PushLimit(bytes);
        if (!s.ParseFromCodedStream(&coded_input)) {
            std::cout << "ParseFromCodedStream() returned false." << std::endl;
            break;
        }
        coded_input.PopLimit(old_limit);
        sv.push_back(s);
        s.Clear();
    }
    
    if (::close(fd)) {
        std::cerr << "Error closing file." << std::endl;
        return false;
    }

    // now convert all fields in samples to vectors;
    QMap<QString, QVector<double>> sv_time_series = to_time_series(sv);
    // compute all meta data for each field
    QMap<QString, gui::MetaData> sv_time_series_meta_data = compute_meta_data(sv_time_series);

    // Insert into map of messages to message data
    {
        QMutexLocker locker(&mutex_);
        proto_messages_[filename] = sv;
        time_series_[filename] = sv_time_series;
        time_series_meta_data_[filename] = sv_time_series_meta_data;
    }

    return true;
}

DataWrangler::~DataWrangler()
{
    // std::cout << "Destructor of instance " << this << " called\n";
    // for (int i = 0; i < filenames_.size(); ++i)
    //     std::cout << filenames_[i].toStdString() << std::endl;
    // google::protobuf::ShutdownProtobufLibrary();
}

QMap<QString, QVector<double>> DataWrangler::to_time_series(const QVector<sample::Sample> & sv)
{
    QMap<QString, QVector<double>> result;

    // Default construct a QVector<double>, then reserve the right amount of
    // space.
    for (const QString & s : fieldnames_) {
        result[s] = QVector<double>();
        result[s].reserve(sv.size());
    }

    // 
    for (const sample::Sample & s : sv) {
        result["time"].push_back(s.system_time() * constants::system_timer_seconds_per_count);
        result["acc_x"].push_back(s.mpu6050().accelerometer_x());
        result["acc_y"].push_back(s.mpu6050().accelerometer_y());
        result["acc_z"].push_back(s.mpu6050().accelerometer_z());
        result["temp"].push_back(s.mpu6050().temperature());
        result["gyro_x"].push_back(s.mpu6050().gyroscope_x());
        result["gyro_y"].push_back(s.mpu6050().gyroscope_y());
        result["gyro_z"].push_back(s.mpu6050().gyroscope_z());
        result["rear_wheel"].push_back(s.encoder().rear_wheel());
        result["rear_wheel_rate"].push_back(s.encoder().rear_wheel_rate());
        result["steer"].push_back(s.encoder().steer());
        result["steer_rate"].push_back(s.encoder().steer_rate());
        result["T_rw"].push_back(s.motor_torque().rear_wheel());
        result["T_rw_desired"].push_back(s.motor_torque().desired_rear_wheel());
        result["T_s"].push_back(s.motor_torque().steer());
        result["T_s_desired"].push_back(s.motor_torque().desired_steer());
        result["v"].push_back(-s.encoder().rear_wheel_rate() * constants::wheel_radius);
        result["v_c"].push_back(-s.set_point().theta_r_dot() * constants::wheel_radius);
        result["yr_c"].push_back(s.set_point().yaw_rate());
        result["theta_r_dot_lb"].push_back(s.estimate().theta_r_dot_lower());
        result["theta_r_dot_ub"].push_back(s.estimate().theta_r_dot_upper());
        result["lean_est"].push_back(s.estimate().lean());
        result["steer_est"].push_back(s.estimate().steer());
        result["lean_rate_est"].push_back(s.estimate().lean_rate());
        result["steer_rate_est"].push_back(s.estimate().steer_rate());
        result["yaw_rate_est"].push_back(s.estimate().yaw_rate());
    }

    return result;
}

QMap<QString, gui::MetaData>
DataWrangler::compute_meta_data(const QMap<QString, QVector<double>> & ts_data)
{
    QMap<QString, gui::MetaData> result;
    for (const auto & field : ts_data.keys())
        result.insert(field, MetaData(ts_data[field]));

    return result;
}

} // namespace gui


