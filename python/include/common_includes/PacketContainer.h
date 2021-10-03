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

#ifndef __PACKET_CONTAINER_H__
#define __PACKET_CONTAINER_H__

#include <stdint.h>

/** 
 * @file PacketContainer.h
*/

#define IP_HEADER_LENGTH 20
#define UDP_HEADER_LENGTH 8 
#define MAX_UDP_PAYLOAD_SIZE (UINT16_MAX - (IP_HEADER_LENGTH + UDP_HEADER_LENGTH))


namespace invz {

	struct PacketContainerHeader
	{
		uint64_t timestamp = UINT64_MAX;
		uint32_t length = 0;
	};

	struct PacketContainer : PacketContainerHeader
	{
		std::unique_ptr<uint8_t[]> payload = nullptr;

		PacketContainer()
		{
			payload = std::make_unique<uint8_t[]>(MAX_UDP_PAYLOAD_SIZE);
		}

		PacketContainer(size_t payloadSize)
		{
			if (payloadSize > 0)
			{
				payload = std::make_unique<uint8_t[]>(payloadSize);
				if (!payload)
				{
					throw std::runtime_error("Failed allocate payload of PacketContainer");
				}
			}


		}

		/* Copy constructor */
		PacketContainer(PacketContainer& other)
		{
			timestamp = other.timestamp;
			length = other.length;
			payload = std::make_unique<uint8_t[]>(other.length);
			memcpy(payload.get(), other.payload.get(), other.length);
		}

		~PacketContainer()
		{
			timestamp = UINT64_MAX;
			length = 0;
			payload.reset();
		}

	};
}

#endif /* __PACKET_CONTAINER_H__ */