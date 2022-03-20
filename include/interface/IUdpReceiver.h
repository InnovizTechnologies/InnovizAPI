// Copyright 2021 Innoviz Technologies
//
// Licensed under the Innoviz Open Dataset License Agreement (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://github.com/InnovizTechnologies/InnovizAPI/blob/master/LICENSE.md
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __IUDP_RECEIVER_H__
#define __IUDP_RECEIVER_H__

/**
* @file IUdpReceiver.h
* @brief File containing classes and structures required to
* recept and parse the UDP datapacket sent by the target sensor
* The udp communication is responsible of sending the point clouds
* data
*/

#include "common_includes/PacketContainer.h"
#include "common_includes/invz_types.h"

namespace invz {

	class INVZ_API IUdpReceiver
	{
	public:
		virtual ~IUdpReceiver() = default;
		virtual bool CloseConnection() = 0;
		virtual bool StartListening() = 0;
		virtual void RegisterCallback(std::function<bool(uint16_t, PacketContainer*)> callback) = 0;
	};

	INVZ_API IUdpReceiver* UdpReceiverInit(uint32_t marker, const std::string& ipAddress, int lidarPort, uint32_t api_log_severity_level, const std::string& multicastIP, const std::string& networkAdapterIp);

}
#endif /*__IUDP_RECEIVER_H__*/