/* ScitosHead.h
 *      Author: chris burbridge
 *
 * Module for interfacing the HRI head module
 * See http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#HeadG5_Section
 */
#ifndef SCITOHEAD_H_
#define SCITOHEAD_H_

#include <scitos_driver/ScitosModule.h>

class ScitosHead: public ScitosModule {
public:
	static ScitosModule*  Create() {
		return new ScitosHead();
	}

	void initialize();
private:
	ScitosHead();
};

#endif /* SCITOSHEAD_H_ */
