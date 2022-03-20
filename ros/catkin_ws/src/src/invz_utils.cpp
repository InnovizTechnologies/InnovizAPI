#include "invz_utils.h"

// std
#include <chrono>
#include <thread>

// using
using namespace std::chrono;

// constants
static constexpr size_t GET_ATTRIBUTES_INTERVAL_MILLI = 100;

namespace invz_utils
{
    vector<FrameDataAttributes> getAttributes(IReader& ireader, size_t maxAttributes)
    {
        std::vector<FrameDataAttributes> output(maxAttributes);
        size_t nofAttributes = output.size();
        auto res = ireader.GetFrameDataAttributes(output.data(), nofAttributes);
        if (res.error_code != ERROR_CODE_OK)
        {
            throw std::runtime_error("failed to get attributes - error code: " + std::to_string(res.error_code) +
                ", error_message: " + res.error_message);
        }
        
        output.resize(nofAttributes);
        return output;        
    }

    vector<FrameDataAttributes> getAttributesTimeout(IReader& ireader, size_t maxAttributes, unsigned int timeout)
    {
        if (timeout == 0)
        {
            return getAttributes(ireader, maxAttributes);
        }
        else
        {
            auto start = steady_clock::now();
            while (duration_cast<milliseconds>(steady_clock::now() - start).count() < timeout)
            {
                try 
                {
                    return getAttributes(ireader, maxAttributes);
                } 
                catch (const std::runtime_error& e)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(GET_ATTRIBUTES_INTERVAL_MILLI));
                }
            }

            // failed to get attributes in the specified time
            throw std::runtime_error("failed to get attributes in the specified time!");
        }
    }

    vector<FrameDataUserBuffer> createBuffers(const vector<FrameDataAttributes>& attributes, const set<GrabType>& requestedBuffers)
    {
        // init output and get references
        vector<FrameDataUserBuffer> output;

        // fill output
        output.reserve(requestedBuffers.size());
        for (const auto& attribute: attributes)
        {
            // check if attribute is requested
            if (requestedBuffers.count(attribute.known_type))
            {
                output.emplace_back(attribute);
            }
        }

        return output;
    }

    map<GrabType, FrameDataUserBuffer*> mapBuffers(vector<FrameDataUserBuffer>& buffers)
    {
        // init output
        map<GrabType, FrameDataUserBuffer*> output;

        // fill output
        for (auto& buffer : buffers)
        {
            output[buffer.dataAttrs.known_type] = &buffer;
        }

        return output;
    }

    void closeFileReader(IReader* iReader)
    {
        if (iReader)
        {
            FileReaderClose(iReader);
        }
    }

    void closeDeviceInterface(IReader* iReader)
    {
        if (iReader)
        {
            IDevice& device = dynamic_cast<IDevice&>(*iReader);
            DeviceClose(&device);
        }
    }
}
