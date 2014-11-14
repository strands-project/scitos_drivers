/* ScitosEBC.h
 *      Author: chris burbridge
 *
 * Module for EBC power board control.
 * http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#EBC_Section
 */

#ifndef SCITOSEBC_H_
#define SCITOSEBC_H_

#include "scitos_mira/ScitosDrive.h"
#include "scitos_mira/ScitosModule.h"
#include "scitos_mira/EBCParametersConfig.h"
#include "scitos_mira/ScitosG5.h"

#include <dynamic_reconfigure/server.h>

class ScitosEBC: public ScitosModule {
public:
	static ScitosModule*  Create() {
		return new ScitosEBC();
	}

	void initialize();

	void reconfigure_callback(scitos_mira::EBCParametersConfig &config, uint32_t level);
private:
	ScitosEBC();
	dynamic_reconfigure::Server<scitos_mira::EBCParametersConfig> reconfigure_srv_;

};

#endif /* SCITOSEBC_H_ */
