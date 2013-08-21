// ============================================================================
// 
//         Author:  Dale Lukas Peterson (dlp), hazelnusse@gmail.com
// 
//    Description:  main point of entry for data explorer tool
// 
// ============================================================================

#include <iostream>
#include <QApplication>
#include <QVector>
#include <QString>
#include "mainwindow.h"
#include "qcustomplot.h"

int main(int argc, char *argv[])
{
//#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
//    QApplication::setGraphicsSystem("raster");
//#endif

    // argc and argv can be changed by QApplication constructor
    QApplication a(argc, argv);

    if (argc < 2) {
        std::cout << "Please supply data files for exploration." << std::endl;
        return EXIT_FAILURE;
    }

    QVector<QString> data_filenames; data_filenames.reserve(argc - 1);
    for (int i = 1; i < argc; ++i)
        data_filenames.push_back(argv[i]);

    gui::MainWindow w(data_filenames);
    w.show();
  
    return a.exec();
}

