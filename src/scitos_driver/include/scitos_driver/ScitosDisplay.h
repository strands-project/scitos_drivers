/* ScitosDisplay.h
 *      Author: chris burbridge
 *
 * Module for interfacing all the mini embedded status display topics, services and params.
 * http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#StatusDisplay_Section
 */

#ifndef SCITODISPLAY_H_
#define SCITODISPLAY_H_

#include <scitos_driver/ScitosModule.h>

class ScitosDisplay: public ScitosModule {
public:
	static ScitosModule*  Create() {
		return new ScitosDisplay();
	}

	void initialize();
private:
	ScitosDisplay();
};

#endif
