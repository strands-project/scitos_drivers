/* ScitosDisplay.h
 *      Author: chris burbridge
 *
 * Module for interfacing all the mini embedded status display topics, services and params.
 * http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#StatusDisplay_Section
 */

#ifndef SCITODISPLAY_H_
#define SCITODISPLAY_H_

#include <scitos_mira/ScitosModule.h>
#include <dynamic_reconfigure/server.h>
#include <scitos_mira/DisplayParametersConfig.h>

class ScitosDisplay: public ScitosModule {
public:
	static ScitosModule*  Create() {
		return new ScitosDisplay();
	}

	void initialize();
	
	void reconfigure_callback(scitos_mira::DisplayParametersConfig &config, uint32_t level);
	void menu_data_callback(mira::ChannelRead<uint8> data);

private:
	ScitosDisplay();
	dynamic_reconfigure::Server<scitos_mira::DisplayParametersConfig> reconfigure_srv_;
	ros::Publisher display_data_pub_;
};

#endif
