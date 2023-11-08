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

#include "Messages.h"

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
	void ForwardMissiongControlMsg(const uint8_t* data, const uint8_t length);
	void ProcessRadioMessage(uint8_t* msg, uint16_t length);

	RFM69 radio;
	enum class State : uint8_t {

	};
};


#endif /* RADIOCOMMS_H_ */
