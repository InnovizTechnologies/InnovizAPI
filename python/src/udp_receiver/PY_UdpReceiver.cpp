///////////////////////////////////////////////////////////
//  PY_UdpReceiver.cpp
//  Implementation of the Class PY_UdpReceiver
//  Created on:      09-Aug-2021 2:13:05 PM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#include "PY_UdpReceiver.h"


 PY_UdpReceiver::PY_UdpReceiver(const std::string ipAddress, const int lidarPort, uint32_t marker, uint32_t log_severity, 
	 const std::string& multicast_ip, const std::string& networkAdapterIp)
 {
	 m_udpR.reset(invz::UdpReceiverInit(marker, ipAddress, lidarPort, log_severity, multicast_ip, networkAdapterIp));
	 udpPacketCallbackCpp = [&](uint16_t port, invz::PacketContainer* pc) -> bool {
		 /* Acquire GIL before calling Python code */
		 py::gil_scoped_acquire acquire;
		 PyPacketContainer py_container(*pc);
		 udpPacketCallbackPy(port, py_container);
		 return true;
	 };
 }

 PY_UdpReceiver::~PY_UdpReceiver()
 {
	 if (m_udpR)
	 {
		 m_udpR->CloseConnection();
		 m_udpR.release();
	 }
 }

 void PY_UdpReceiver::RegisterCallback(std::function<bool(uint16_t, PyPacketContainer&)> callback)
 {
	 udpPacketCallbackPy = std::bind(callback, std::placeholders::_1, std::placeholders::_2);
	 m_udpR->RegisterCallback(udpPacketCallbackCpp);
 }

 bool PY_UdpReceiver::StartListening()
 {

	 return m_udpR->StartListening();

 }

 bool PY_UdpReceiver::StopListening()
 {

	 return m_udpR->CloseConnection();
 }
