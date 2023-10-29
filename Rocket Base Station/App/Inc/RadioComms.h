/*
 * RadioComms.h
 *
 *  Created on: Oct 10, 2021
 *      Author: johnt
 */

#ifndef RADIOCOMMS_H_
#define RADIOCOMMS_H_

#include <Rfm69.h>
#include "cmsis_os2.h"

class RadioComms
{
public:
	RadioComms();
	~RadioComms();

	void Run();

private:
	bool Initialize();
	void HandleRadioPacketReceived();
	void HandleRadioSendStatusMsg();
	void HandleRadioInterrupt();
	void HandleRadioTick();
	void HandleSendStartDataStream();

	RFM69 radio;
	enum class State : uint8_t {

	};
};


#endif /* RADIOCOMMS_H_ */
