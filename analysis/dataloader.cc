#include <fcntl.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>

#include "sample.pb.h"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include "dataloader.h"

namespace gui {

// Have to define the mutex
QMutex DataLoader::mutex_;

DataLoader::DataLoader(QMap<QString, QVector<sample::Sample>> & data_set,
                       QVector<QString> filenames)
    : data_set_(data_set),
      filenames_{filenames}
{

}

bool DataLoader::operator()(QString const & filename)
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
    std::string output(filename.toLocal8Bit().constData());
    output += ": " + std::to_string(sv.size()) + " samples read.\n";
    std::cout << output;
    {
        QMutexLocker locker(&mutex_);
        data_set_[filename] = sv;
    }
    if (::close(fd)) {
        std::cerr << "Error closing file." << std::endl;
        return false;
    }

    return true;
}

DataLoader::~DataLoader()
{
    google::protobuf::ShutdownProtobufLibrary();
}

} // namespace gui

