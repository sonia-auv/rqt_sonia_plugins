
#include <QApplication>
#include "main_window/can_client.h"
#include <ros/node_handle.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "can_client");

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    QApplication a(argc, argv);
    CanClient w(0);
    w.show();

    //** ros spinner */
    // spin sur 2 threads seulement
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // start Qt
    a.exec();
    spinner.stop();

    return EXIT_SUCCESS;
}
