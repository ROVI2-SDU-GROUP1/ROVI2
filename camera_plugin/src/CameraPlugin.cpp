#include "CameraPlugin.hpp"
#include <cv_bridge/cv_bridge.h>
#include <math.h>
 
#include <rws/RobWorkStudio.hpp>
#include <qtimer.h>
#include <QPushButton>


using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;

using namespace rws;


CameraPlugin::CameraPlugin():
    RobWorkStudioPlugin("camera plugin", QIcon(":/pa_icon.png"))
{
	setupUi(this);
	char** argv = NULL;
        int argc = 0;
        ros::init(argc, argv,"camera_plugin");

        _timer = new QTimer(this);
        connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

        _qtRos = new QtROS();

        connect(this, SIGNAL(quitNow()), _qtRos, SLOT(quitNow()));
        
	qRegisterMetaType<cv::Mat>("cv::Mat");
        connect(_qtRos, SIGNAL(newImage(cv::Mat)), this, SLOT(newImage(cv::Mat)));
               
}

CameraPlugin::~CameraPlugin()
{
}


void CameraPlugin::newImage(cv::Mat image){
        
        cv::Mat temp; // make the same cv::Mat
        cvtColor(image, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
        QImage dest((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
        QImage dest2(dest);
        dest2.detach(); // enforce deep copy

        QPixmap p = QPixmap::fromImage(dest2);
        unsigned int maxW = 400;
        unsigned int maxH = 800;
        _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

}


void CameraPlugin::initialize() {
	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&CameraPlugin::stateChangedListener, this, _1), this);
}

void CameraPlugin::open(WorkCell* workcell)
{
	_wc = workcell;
	_state = _wc->getDefaultState();

}


void CameraPlugin::close() {
     _wc = NULL;
}

cv::Mat CameraPlugin::toOpenCVImage(const Image& img) {
	cv::Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void CameraPlugin::btnPressed() {
	QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Button 0 start\n";
                _qtRos->start();
                
	} else if(obj==_btn1){
		log().info() << "Button 1\n";
                emit quitNow();
	}
}

void CameraPlugin::timer() {
        _timer->stop();
}

void CameraPlugin::stateChangedListener(const State& state) {
	_state = state;
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(CameraPlugin);
#endif
