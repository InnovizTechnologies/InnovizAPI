from innopy.api import DeviceInterface, TLV, AcpHeader
import numpy as np

DTYPE_SET_LIDAR_MODE = np.dtype([('mode', 'u1'),
                                 ('sub_mode', 'u1'),
                                 ('reset', 'u1'),
                                 ('reserved', 'u1')])

DTYPE_LIDAR_MODE_RESPONSE = np.dtype([('return_code', 'u4')])


class SetLidarMode_TLV():
    def __init__(self):
        self.request_dtype = DTYPE_SET_LIDAR_MODE
        self.response_dtype = DTYPE_LIDAR_MODE_RESPONSE
        self.SET_LIDAR_MODE_TLV = 0x00080005
        self.normal_low_power_mode = {"mode": 2, "sub_mode": 2, "reset": 1}
        self.normal_high_power_mode = {"mode": 2, "sub_mode": 0, "reset": 1}
        self.standby_mode = {"mode": 3, "sub_mode": 0, "reset": 0}
        self.degradation_user_mode = {"mode": 5, "sub_mode": 5, "reset": 0}
        self.req = self.request_acp_header()
        config_files_path = '../lidar_configuration_files'
        self.di = DeviceInterface(config_file_name=config_files_path + '/om_config.json', is_connect=False)

    def request_acp_header(self):
        request_acp_header = AcpHeader()
        request_acp_header.options = 0x0e
        request_acp_header.communication_options = 0
        request_acp_header.master_id = 1
        request_acp_header.is_response = 0
        request_acp_header.protocol_version = 1
        return request_acp_header


    def set_lidar_mode(self, mode, sub_mode, reset):
        tlv = TLV(self.SET_LIDAR_MODE_TLV, self.request_dtype.itemsize, np.zeros((1,), dtype=self.request_dtype))
        tlv.value.dtype = self.request_dtype
        tlv.value['mode'] = mode
        tlv.value['sub_mode'] = sub_mode
        tlv.value['reset'] = reset
        response_acp_header, response_tlv, port, request_acp_header_after_send = self.di.send_tlv(self.req, tlv)
        response_tlv.value.dtype = self.response_dtype
        if response_tlv.value['return_code'] == 0:
            print(f"Mode changed successfully\n")
        else:
            print(f"Change mode failed\n")


if __name__ == '__main__':
    tlv_flow = SetLidarMode_TLV()
    mode = tlv_flow.normal_low_power_mode["mode"]
    sub_mode = tlv_flow.normal_low_power_mode["sub_mode"]
    reset = tlv_flow.normal_low_power_mode["reset"]
    tlv_flow.set_lidar_mode(mode, sub_mode, reset)
