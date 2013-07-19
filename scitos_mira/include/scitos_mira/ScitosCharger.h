/* ScitosCharger.h
 *      Author: chris burbridge
 *
 * Module for interfacing all the charger topics, service and params.
 * See http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#ChargerHG2_Section
 */

#ifndef SCITOSCHARGER_H_
#define SCITOSCHARGER_H_

#include <scitos_mira/ScitosModule.h>

//
//#include <fw/Framework.h>
#include <robot/BatteryState.h>
#include <dynamic_reconfigure/server.h>
#include <scitos_mira/ChargerParametersConfig.h>

class ScitosCharger: public ScitosModule {
public:
	static ScitosModule*  Create() {
		return new ScitosCharger();
	}

	void initialize();

	void battery_data_callback(mira::ChannelRead<mira::robot::BatteryState> data);
	void charger_status_callback(mira::ChannelRead<uint8> data);
	void reconfigure_callback(scitos_mira::ChargerParametersConfig &config, uint32_t level);
private:

	ScitosCharger();
	ros::Publisher battery_pub_;
	ros::Publisher charger_pub_;
	dynamic_reconfigure::Server<scitos_mira::ChargerParametersConfig> reconfigure_srv_;
};

#endif /* SCITOSCHARGER_H_ */
