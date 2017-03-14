#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP


#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

#include "ui_CameraPlugin.h"
#include "qtros.h"
#include <ros/ros.h>
#include <opencv/cv.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/models.hpp>

class CameraPlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
public:
	CameraPlugin();
	virtual ~CameraPlugin();

	virtual void open(rw::models::WorkCell* workcell);

	virtual void close();

	virtual void initialize();


private slots:
	void btnPressed();
	void timer();

	void stateChangedListener(const rw::kinematics::State& state);
        void newImage(cv::Mat);

signals:
	void quitNow();

private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

	QTimer* _timer;
        QtROS *_qtRos;
	
	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;

};

#endif /*RINGONHOOKPLUGIN_HPP_*/
