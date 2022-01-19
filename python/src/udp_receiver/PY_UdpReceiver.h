///////////////////////////////////////////////////////////
//  PY_UdpReceiver.h
//  Implementation of the Class PY_UdpReceiver
//  Created on:      09-Aug-2021 2:13:05 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#ifndef __PY_UDP_RECEIVER_H__
#define __PY_UDP_RECEIVER_H__

#include "../common_files/PY_CommonUtils.h"


class PY_UdpReceiver
{
public:
	PY_UdpReceiver(const std::string ipAddress, const int lidarPort, uint32_t marker, uint32_t log_severity, 
		const std::string& multicast_ip, const std::string& networkAdapterIp);
	~PY_UdpReceiver();
	void RegisterCallback(std::function<bool(uint16_t, PyPacketContainer&)> callback);
	bool StartListening();
	bool StopListening();

private:
	std::unique_ptr<invz::IUdpReceiver> m_udpR = nullptr;
	std::function<bool(uint16_t, PacketContainer*)> udpPacketCallbackCpp;
	std::function<void(uint16_t, PyPacketContainer&)> udpPacketCallbackPy;
};
#endif //  __PY_UDP_RECEIVER_H__
