/** 
 * QtThread based class encapsulating the ros basics,
 * i.e., init, node handle creation, spining and quitting.
 * To quit via qt, connect the quitNow slot to, e.g., 
 * the aboutToQuit-Signal of qapplication.
 */
#ifndef QT_ROS_H
#define QT_ROS_H
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <QThread>
#include <QObject>

class QtROS : public QThread {
  Q_OBJECT

  public:
    ///Note: The constructor will block until connected with roscore
    ///Instead of ros::spin(), start this thread with the start() method
    ///to run the event loop of ros
    QtROS();
    //ros::NodeHandle getNodeHandle(){ return *n; }
    /// This method contains the ROS event loop. Feel free to modify 
    void run();
  public slots:
    ///Connect to aboutToQuit signals, to stop the thread
    void quitNow();
  signals:
    ///Triggered if ros::ok() != true
    void rosQuits();
    void newImage(cv::Mat);
  private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool quitfromgui;
    void process();
    sensor_msgs::ImageConstPtr _imageIn;
    cv_bridge::CvImagePtr _imageOut;
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Subscriber _image_sub;
    
};
#endif

