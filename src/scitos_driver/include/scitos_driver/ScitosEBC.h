/* ScitosEBC.h
 *      Author: chris burbridge
 *
 * Module for EBC power board control.
 * http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#EBC_Section
 */

#ifndef SCITOSEBC_H_
#define SCITOSEBC_H_

#include <scitos_driver/ScitosModule.h>

#include <dynamic_reconfigure/server.h>
#include <scitos_driver/EBCParametersConfig.h>

class ScitosEBC: public ScitosModule {
public:
	static ScitosModule*  Create() {
		return new ScitosEBC();
	}

	void initialize();

	void reconfigure_callback(scitos_driver::EBCParametersConfig &config, uint32_t level);
private:
	ScitosEBC();
	dynamic_reconfigure::Server<scitos_driver::EBCParametersConfig> reconfigure_srv_;

};

#endif /* SCITOSEBC_H_ */
