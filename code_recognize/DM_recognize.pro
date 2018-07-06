#-------------------------------------------------
#
# Project created by QtCreator 2018-05-25T16:05:48
#
#-------------------------------------------------

QT       += core gui
CONFIG += c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DM_recognize
TEMPLATE = app


SOURCES += main.cpp\
        dm_recognize.cpp \


HEADERS  += dm_recognize.h

FORMS    += dm_recognize.ui

include(QZXing/QZXing.pri)


INCLUDEPATH += /usr/local/include/          \
               /usr/local/include/opencv    \
               /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_core.so        \
        /usr/local/lib/libopencv_calib3d.so     \
        /usr/local/lib/libopencv_features2d.so  \
        /usr/local/lib/libopencv_flann.so       \
        /usr/local/lib/libopencv_highgui.so     \
        /usr/local/lib/libopencv_imgcodecs.so   \
        /usr/local/lib/libopencv_imgproc.so     \
        /usr/local/lib/libopencv_ml.so          \
        /usr/local/lib/libopencv_objdetect.so   \
        /usr/local/lib/libopencv_photo.so       \
        /usr/local/lib/libopencv_shape.so       \
        /usr/local/lib/libopencv_stitching.so   \
        /usr/local/lib/libopencv_superres.so    \
        /usr/local/lib/libopencv_video.so       \
        /usr/local/lib/libopencv_videoio.so     \
        /usr/local/lib/libopencv_videostab.so   \
        /usr/local/lib/libopencv_viz.so         \
        /usr/local/lib/libwiringPi.so           \
        /usr/local/lib/libwiringPiDev.so        \
        /usr/local/lib/libdmtx.so

DISTFILES +=
