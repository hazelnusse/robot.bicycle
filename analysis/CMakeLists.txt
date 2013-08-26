include_directories(${PROTOBUF_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS sample.proto)

set(QT_HEADERS mainwindow.h
               qcustomplot.h)
qt4_wrap_cpp(QT_HEADERS_MOC ${QT_HEADERS})

add_executable(data-explorer
               datawrangler.cc
               main.cc
               mainwindow.cc
               qcustomplot.cpp
               ${QT_HEADERS_MOC}
               ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(data-explorer
                      bicycle
                      ${PROTOBUF_LIBRARIES}
                      ${QT_LIBRARIES}
                      ${FFTW_LIB}
                      ${CMAKE_THREAD_LIBS_INIT})
