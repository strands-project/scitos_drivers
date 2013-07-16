#include "scitos_mira/ScitosEBC.h"

#include "scitos_mira/ScitosG5.h"

ScitosEBC::ScitosEBC() : ScitosModule(std::string ("EBC")), reconfigure_srv_(name_) {

}

void ScitosEBC::initialize() {
	reconfigure_srv_.setCallback(boost::bind(&ScitosEBC::reconfigure_callback, this, _1, _2));

}

void ScitosEBC::reconfigure_callback( scitos_mira::EBCParametersConfig& config, uint32_t level) {
	ROS_DEBUG("Reconfigure request on ScitosEBC module.");
	//Set the MIRA parameters to what was selected...
	// Port 0 
	if (config.Port0_5V_Enabled)
	  set_mira_param_("EBC7.Port0_5V.Enabled","true");
	else
	  set_mira_param_("EBC7.Port0_5V.Enabled","false");
	set_mira_param_("EBC7.Port0_5V.MaxCurrent",std::to_string(config.Port0_5V_MaxCurrent));
				   
	if (config.Port0_12V_Enabled)
	  set_mira_param_("EBC7.Port0_12V.Enabled","true");
	else
	  set_mira_param_("EBC7.Port0_12V.Enabled","false");
	set_mira_param_("EBC7.Port0_12V.MaxCurrent",std::to_string(config.Port0_12V_MaxCurrent));
	
	if (config.Port0_24V_Enabled)
	  set_mira_param_("EBC7.Port0_24V.Enabled","true");
	else
	  set_mira_param_("EBC7.Port0_24V.Enabled","false");
	set_mira_param_("EBC7.Port0_24V.MaxCurrent",std::to_string(config.Port0_24V_MaxCurrent));

	// Port 1
	if (config.Port1_5V_Enabled)
	  set_mira_param_("EBC7.Port1_5V.Enabled","true");
	else
	  set_mira_param_("EBC7.Port1_5V.Enabled","false");
	set_mira_param_("EBC7.Port1_5V.MaxCurrent",std::to_string(config.Port1_5V_MaxCurrent));
	
	if (config.Port1_12V_Enabled)
	  set_mira_param_("EBC7.Port1_12V.Enabled","true");
	else
	  set_mira_param_("EBC7.Port1_12V.Enabled","false");
	set_mira_param_("EBC7.Port1_12V.MaxCurrent",std::to_string(config.Port1_12V_MaxCurrent));

	if (config.Port1_24V_Enabled)
	  set_mira_param_("EBC7.Port1_24V.Enabled","true");
	else
	  set_mira_param_("EBC7.Port1_24V.Enabled","false");
	set_mira_param_("EBC7.Port1_24V.MaxCurrent",std::to_string(config.Port1_24V_MaxCurrent));

}
