#ifndef INVZ_UTILS_H
#define INVZ_UTILS_H

// std
#include <vector>
#include <map>
#include <set>
#include <algorithm>

// invz
#include "interface/FileReaderApi.h"
#include "interface/DeviceApi.h"
#include "common_includes/invz_types.h"
#include "common.h"

namespace invz_utils
{
    // using
    using std::vector;
    using std::map;
    using std::set;
    using namespace invz;

    // get attributes from IReader
    vector<FrameDataAttributes> getAttributes(IReader& ireader, size_t maxAttributes);

    // get attributes from IReader with timeout - intended to be used with DeviceInterface
    vector<FrameDataAttributes> getAttributesTimeout(IReader& ireader, size_t maxAttributes, unsigned int timeout);

    // create requested buffers from attributes
    vector<FrameDataUserBuffer> createBuffers(const vector<FrameDataAttributes>& attributes, const set<GrabType>& requestedBuffers);

    // map grab type to buffer
    map<GrabType, FrameDataUserBuffer*> mapBuffers(vector<FrameDataUserBuffer>& buffers);

    // close file reader if not nullptr
    void closeFileReader(IReader* iReader);

    // close file reader if not nullptr
    void closeDeviceInterface(IReader* iReader);

    // get keys of map as set
    template<typename MapType>
    set<typename MapType::key_type> mapKeys(const MapType& m)
    {
        set<typename MapType::key_type> output;
        std::transform(m.begin(), m.end(),
            std::inserter(output, output.end()),
            [](auto pair){ return pair.first; });

        return output;
    }

}

#endif
