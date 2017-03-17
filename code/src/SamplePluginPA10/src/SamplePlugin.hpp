#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

//#include "../build/ui_SamplePlugin.h"
#include <SamplePluginPA10/ui_SamplePlugin.h>

#include <opencv2/opencv.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/rw.hpp>

#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
	Q_OBJECT
	Q_INTERFACES( rws::RobWorkStudioPlugin )

	public:
		SamplePlugin();
		virtual ~SamplePlugin();
		virtual void open(rw::models::WorkCell* workcell);
		virtual void close();
		virtual void initialize();

	private slots:
		void btnPressed();
		void timer();
		void stateChangedListener(const rw::kinematics::State& state);

	private:
		static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

		QTimer* _timer;

		rw::models::WorkCell::Ptr _wc;
		rw::kinematics::State _state;
		rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
		rwlibs::simulation::GLFrameGrabber* _framegrabber;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/

