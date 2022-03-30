import math
import random
from enum import Enum
from time import sleep
import numpy as np
from innopy.api import DeviceInterface, AcpHeader, TLV


DTYPE_MEMS_PITCH_STATUS_RESPONSE= np.dtype([('return_status', 'u1'),
                                            ('mems_state', 'u1'),
                                            ('time_left', 'u2'),
                                            ('pitch_current', 'i2'),
                                            ('pitch_target', 'i2'),
                                            ('pitch_max', 'i2'),
                                            ('pitch_min', 'i2'),
                                            ('reserved', ('u1', 8))])

DTYPE_SET_MEMS_PITCH_STATUS_TLV = np.dtype([('mode', 'u1'),
                                            ('relative_abs_angle', 'u1'),
                                            ('required_angle_change', 'i2'),
                                            ('preset', 'u1'),
                                            ('reserved_2', ('u1', 7))])

DTYPE_GET_MEMS_PITCH_STATUS_TLV = np.dtype([('reserved', ('u1', 8))])

class ChangeMode(Enum):
    IMMEDIATE = 0
    GRADUAL = 1

class OffsetType(Enum):
    RELATIVE = 0
    ABSOLUTE = 1
    PRESET = 2

class Preset(Enum):
    MAX = 0
    MIN = 1
    NOMINAL = 2
    LAST = 3

def request_acp_header():
    request_acp_header = AcpHeader()
    request_acp_header.options = 0x0e
    request_acp_header.communication_options = 0
    request_acp_header.master_id = 1
    request_acp_header.is_response = 0
    request_acp_header.protocol_version = 1
    return request_acp_header

def calc_angle_degree(angle_digital):
    #Conversion for optical angle
    angle_degree = 2 * math.atan(angle_digital / (2 ** 18)) * 57.2958
    return angle_degree


class ChangeMemsPitch_TLV():
    def __init__(self, di):
        self.request_dtype = DTYPE_SET_MEMS_PITCH_STATUS_TLV
        self.response_dtype = DTYPE_MEMS_PITCH_STATUS_RESPONSE
        self.CHANGE_MEMS_PITCH_TLV = 0x00080030
        self.req = request_acp_header()
        self.di = di

    def create_set_mems_pitch_status_tlv(self):
        tlv = TLV(self.CHANGE_MEMS_PITCH_TLV, self.request_dtype.itemsize, np.zeros((0,), dtype=self.request_dtype))
        tlv.value.dtype = self.request_dtype
        tlv.value['mode'] = ChangeMode.GRADUAL.value
        tlv.value['relative_abs_angle'] = OffsetType.ABSOLUTE.value
        required_angle_change = 1.5
        target_angle_digital = int(math.tan((required_angle_change / 57.2958) / 2) * (2**18))
        tlv.value['required_angle_change'] = target_angle_digital
        #This parameter relevant when relative_abs_angle is PRESET
        tlv.value['preset'] = Preset.MAX.value
        return tlv

    def set_mems_pitch_status(self):
        try:
            tlv_g = self.create_set_mems_pitch_status_tlv()
            response_acp_header, response_tlv, port, request_acp_header_after_send = self.di.send_tlv(self.req, tlv_g)
            response_tlv.value.dtype = self.response_dtype
            pitch_current = calc_angle_degree(response_tlv.value['pitch_current'])
            pitch_target = calc_angle_degree(response_tlv.value['pitch_target'])
            print('Set mems pitch status response:')
            print( f"Mems pitch status:\n status: {response_tlv.value['return_status']}, mems state: {response_tlv.value['mems_state']}, time left: {response_tlv.value['time_left']}, "
                f"pitch current: {pitch_current}, pitch target: {pitch_target}")
            return response_tlv
        except Exception as e:
            print(e)


class GetPitchChangeStatus_TLV():
    def __init__(self, di):
        self.request_dtype = DTYPE_GET_MEMS_PITCH_STATUS_TLV
        self.response_dtype = DTYPE_MEMS_PITCH_STATUS_RESPONSE
        self.GET_PITCH_CHANGE_STATUS_TLV = 0x00080031
        self.req = request_acp_header()
        self.di = di

    def create_get_mems_pitch_status_tlv(self):
        value = np.zeros((0,), dtype=self.request_dtype)
        get_mems_pitch_status_tlv = TLV(self.GET_PITCH_CHANGE_STATUS_TLV, self.request_dtype.itemsize, value)
        return get_mems_pitch_status_tlv

    def get_mems_pitch_status(self):
        try:
            tlv_g = self.create_get_mems_pitch_status_tlv()
            response_acp_header, response_tlv, port, request_acp_header_after_send = self.di.send_tlv(self.req, tlv_g)
            response_tlv.value.dtype = self.response_dtype
            mems_pitch_status = [response_tlv.value['return_status'], response_tlv.value['mems_state'],
                                 response_tlv.value['time_left'], response_tlv.value['pitch_current'],
                                 response_tlv.value['pitch_target'], response_tlv.value['pitch_max'],
                                 response_tlv.value['pitch_min'], response_tlv.value['reserved']]
            pitch_current = calc_angle_degree(mems_pitch_status[3])
            pitch_target = calc_angle_degree(mems_pitch_status[4])
            pitch_max = calc_angle_degree(mems_pitch_status[5])
            pitch_min = calc_angle_degree(mems_pitch_status[6])
            print( f'Mems pitch status:\n status:{mems_pitch_status[0]}, mems_state: {mems_pitch_status[1]}, time_left: {mems_pitch_status[2]},'
                f' pitch_current: {pitch_current}, pitch_target: {pitch_target}, pitch_max: {pitch_max}, pitch_min: {pitch_min}')
            return response_tlv
        except Exception as e:
            print(repr(e))


def main():
    config_files_path = '../lidar_configuration_files'
    di = DeviceInterface(config_file_name=config_files_path+'/om_config.json', is_connect=False)
    mems_pitch_status_tlv = GetPitchChangeStatus_TLV(di)
    set_mems_pitch_status_tlv = ChangeMemsPitch_TLV(di)
    print("-------- Check mems pitch status ---------")
    mems_pitch_status_tlv.get_mems_pitch_status()
    print("--------- Set mems pitch status  ----------")
    response_tlv = set_mems_pitch_status_tlv.set_mems_pitch_status()
    time_left = response_tlv.value['time_left']
    while time_left > 0:
        sleep(5)
        mems_pitch_status = mems_pitch_status_tlv.get_mems_pitch_status()
        time_left = mems_pitch_status.value['time_left']


if __name__ == '__main__':
    main()
