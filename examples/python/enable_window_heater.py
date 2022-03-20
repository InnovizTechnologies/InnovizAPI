from enum import Enum
import numpy as np
from innopy.api import DeviceInterface, AcpHeader, TLV


DTYPE_SET_HEATER_POWER_TLV = np.dtype([
    ('power', 'u1'),
    ('reserved', ('u1', 3))
])
DTYPE_GET_HEATER_STATE_VALUE = np.dtype([
    ('mode', 'u1'),
    ('power', 'u1'),
    ('reserved', 'u2')
])
DTYPE_SET_HEATER_POWER_RESPONSE = np.dtype([('return_code', 'u4')])


class HEATER_OM_STATE(Enum):
    HEATER_OM_STATE_OFF = 0
    HEATER_OM_STATE_ON = 1
    HEATER_OM_STATE_PAUSE = 2
    HEATER_OM_STATE_ERROR = 3


def request_acp_header():
    request_acp_header = AcpHeader()
    request_acp_header.options = 0x0e
    request_acp_header.communication_options = 0
    request_acp_header.master_id = 1
    request_acp_header.is_response = 0
    request_acp_header.protocol_version = 1
    return request_acp_header

class SetHeaterPower_TLV():
    def __init__(self, di):
        self.request_dtype = DTYPE_SET_HEATER_POWER_TLV
        self.response_dtype = DTYPE_SET_HEATER_POWER_RESPONSE
        self.SET_HEATER_POWER_TLV = 0x00080003
        self.heater_power = {"POWER_ON": 100, "POWER_OFF": 0}
        self.req = request_acp_header()
        self.di = di

    def create_set_heater_power_tlv(self, power):
        value = np.zeros((1,), dtype=self.request_dtype)
        value['power'] = power
        set_heater_power_tlv = TLV(self.SET_HEATER_POWER_TLV, self.request_dtype.itemsize, value)
        return set_heater_power_tlv

    def set_heater_power(self, power):
        tlv_g = self.create_set_heater_power_tlv(power)
        try:
            response_acp_header, response_tlv, port, request_acp_header_after_send = self.di.send_tlv(self.req, tlv_g)
            response_tlv.value.dtype = self.response_dtype
            print('Heater activation response:')
            if response_tlv.value['return_code'] == 0:
                print(f"Succeeded, return code: {response_tlv.value['return_code']}\n")
            else:
                print(f"Failed, return code: {response_tlv.value['return_code']}\n")
        except Exception as e:
            print(e)

class GetHeaterState_TLV():
    def __init__(self, di):
        self.dtype = DTYPE_GET_HEATER_STATE_VALUE
        self.GET_HEATER_STATE_TLV = 0x00080004
        self.req = request_acp_header()
        self.di = di


    def create_get_heater_state_tlv(self):
        value = np.empty((0,))
        get_heater_power_tlv = TLV(self.GET_HEATER_STATE_TLV, 0, value)
        return get_heater_power_tlv

    def get_heater_state(self):
        tlv_g = self.create_get_heater_state_tlv()
        response_acp_header, response_tlv, port, request_acp_header_after_send = self.di.send_tlv(self.req, tlv_g)
        response_tlv.value.dtype = self.dtype
        print(f"Heater_state: {HEATER_OM_STATE(response_tlv.value['mode']).name}, power: {response_tlv.value['power']}\n")


def heater_activate(di):
    get_heater_tlv = GetHeaterState_TLV(di)
    set_heater_power_tlv = SetHeaterPower_TLV(di)
    print("-------- Check Window Heater State ---------")
    get_heater_tlv.get_heater_state()
    print("############################################\n")
    print("--------- Activate Window Heater  ----------")
    set_heater_power_tlv.set_heater_power(set_heater_power_tlv.heater_power["POWER_ON"])
    print('Final heater state:')
    get_heater_tlv.get_heater_state()

def heater_deactivate(di):
    get_heater_tlv = GetHeaterState_TLV(di)
    set_heater_power_tlv = SetHeaterPower_TLV(di)
    print("-------- Check Window Heater State ---------")
    get_heater_tlv.get_heater_state()
    print("############################################\n")
    print("--------- Deactivate Window Heater  ----------")
    set_heater_power_tlv.set_heater_power(set_heater_power_tlv.heater_power["POWER_OFF"])
    print('Final heater state:')
    get_heater_tlv.get_heater_state()


def main():
    config_files_path = '../lidar_configuration_files'
    di = DeviceInterface(config_file_name=config_files_path + '/om_config.json', is_connect=False)
    heater_activate(di)
    # heater_deactivate(di)


if __name__ == '__main__':
    main()
