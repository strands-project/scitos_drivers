#include "scitos_mira/ScitosDisplay.h"
#include "scitos_mira/ScitosG5.h"
#include <std_msgs/Int8.h>


ScitosDisplay::ScitosDisplay() : ScitosModule(std::string ("Display")), reconfigure_srv_(name_) {
}

void ScitosDisplay::initialize() {
	reconfigure_srv_.setCallback(boost::bind(&ScitosDisplay::reconfigure_callback, this, _1, _2));
	
	display_data_pub_ = robot_->getRosNode().advertise<std_msgs::Int8>("/user_menu_selected", 1);
	robot_->getMiraAuthority().subscribe<uint8>("/robot/StatusDisplayUserMenuEvent", 
			boost::bind(&ScitosDisplay::menu_data_callback, this, _1, 1));

}


void ScitosDisplay::reconfigure_callback( scitos_mira::DisplayParametersConfig& config, uint32_t level) {
	ROS_INFO("Reconfigure request on ScitosDisplay module.");
	//Set the MIRA parameters to what was selected...
	if (config.EnableUserMenu) {
	  set_mira_param_("StatusDisplay.EnableUserMenu",std::string("true"));
	  set_mira_param_("StatusDisplay.UserMenuName",config.UserMenuName);
	  set_mira_param_("StatusDisplay.UserMenuEntryName1",config.UserMenuEntryName1);
	  set_mira_param_("StatusDisplay.UserMenuEntryName2",config.UserMenuEntryName2);
	  set_mira_param_("StatusDisplay.UserMenuEntryName3",config.UserMenuEntryName3);
	} else {
	  set_mira_param_("StatusDisplay.EnableUserMenu",std::string("false"));
	}
}

void ScitosDisplay::menu_data_callback(mira::ChannelRead<uint8> data, int i) {
  std_msgs::Int8 item;
  item.data=data->value();
  display_data_pub_.publish(item);
}
