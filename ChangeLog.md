# Innoviz API Change Log

## v5.6.1
- Falcon FOV resolution support

## v5.5.1
- Python wrapper updates:
	1. Updated connect function to return actual connection level
	2. Fix InvzFormat to return correct value for INVZ5 file format
- VB algo updates:
	1. Disabled 'neighbor pixels reflectivity' logic 
	2. vb_usage_mode set to zero (not in use) by default
- Added 'PIXEL_NOISE' to PixelValidity enum (for future use)
- Mems pitch status TLV (0x50042) structure fix 
- ROS updates:
	1. Fix multiple lidar launch file
	2. Fix sampled timestamp
	
## v5.4.0
- Recording index generation update
- Python wrapper INVZ4FileRawWriter/write_payload 
  added optional parameter - packet_length
  
## v5.3.0
- ROS updates:
	1. Rewrote Device and FileReader nodes - improved performance and syntax
	2. Added option to select timestamp in message header (sampled, received, published)
	3. Fixed an issue with overwriting arguments from command line
	
- INVZ5 protocol version support:
	1. EFileFormat enum update - added E_FILE_FORMAT_INVZ5 
	2. Added GRAB_TYPE_PIXEL_LANE_MARK_TRAILER data type to receive lane mark trailer data buffer
	3. Added 'pulses_fired' field to INVZ2PixelLaneMarkTrailer (GRAB_TYPE_PIXEL_LANE_MARK_TRAILER)
	4. INVZ2PixelMetaData struct update (GRAB_TYPE_SINGLE_PIXEL_META_DATA) - added lane_mark_trailer_available field
	5. AutoTaps update - added Lane mark trailer score, reference, ga_index and pulses fired taps per lrf (use attached AutoTaps.json file)
	6. Updated CSampleFrameMetaData struct to fit TLV 0x17 struct update in INVZ5 protocol
	
- New python wrapper DevieInterface examples:
	1. Change Lidar power mode
	2. Change Mems Pitch angle
	3. Enable window heater
	
- VC2 (Runtime logs) update:
	Handle Log message split between two ACP packets
	
- OC SI TLV Support (0x101004) - GRAB_TYPE_OC_OUTPUT_SI
- BWD SI TLV Support (0x101005) - GRAB_TYPE_FOV_OUTPUT_SI
- OM Indications TLV (0x50039) support in python wrapper - GRAB_TYPE_OM_INDICATIONS
- Mems pitch status TLV (0x50042) support - GRAB_TYPE_MEMS_PITCH_STATUS
- Removed FrameMetaData struct from invz_types.h (PRO related)
- DeviceInterface in python wrapper constructor update:
	Added 'require_data_attr' parameter (optional - set to 'True' by default)
	to enable fast connect when no PC required.


## v4.14.0
- GetPacket (from file) interface update enabling VC selection

## v4.13.2
- GRAB_TYPE_PC_PLUS_SUMMATION EOS (deprecated)
- GRAB_TYPE_PC_PLUS_METADATA_48K TLV update (aligned to CVApp structer)
- Parsing VC2 (RuntimeLogs) payload size fix
- ROS launch files updates
- Imptroved FileReader performance 

## v4.12.0
- INVZ4_7 protocol version support:
	1. ultra_short_range added to Macro pixel metadata
	2. ultra_short_range tap added to AutoTaps (use the new AutoTaps.json attached)
- Python wrapper:
	1. GRAB_TYPE_MACRO_PIXEL_META_DATA dtype update (unified struct to support different protocols)

## v4.11.0
- SignGantry TLV support (0x100029)
- VB algo updates:
	1. PCF decompressed reflectivity
	2. Pre processing (set invalid pixels)
- Python wrapper seperate modules
- PC+ detections variable buffer size support (according to PC+ metadata)
- DeviceInterface updates:
	1. Loopback support 
	2. Enable network device specifying by "network_adapter_ip" configuration parameter 
- IS_VALID buffers end of support
	
## v4.9.4
- Added limit to UDP packet queue

## v4.9.3
- VC2 FileReader fix

## v4.9.1
- ROS updates:
	ROS publisher update - reducing points that do not meet the FPA threshold
	Adding reference for using two lidars in ROS
- logs.json and log_modules.json configurable path
- AutoTaps.json configurable path
- Sensor POS TLV update
- RBD TLV support (0x101012)
- OM Indications TLV support (0x50039)

## v4.8.2
- INVZ4_6 protocol full support (including AutoTaps)
- GlareInFov TLV + AutoTaps support

## v4.6.3
- INS Signals TLV support (debug port)
- INVZ4_6 protocol number support
- VB performance improvements

## v4.4.1
- ROS updates: 
	1. Support stream recording in Device Interface
- Changed package layout
- Vertical Blooming filter support

## v4.1.0
- ROS updates: 
	1. Change node names to match standard ROS conventions
	   Device_Publisher -> Innoviz_one_Device
	   FileReader_Publisher -> Innoviz_one_FileReader
	2. Change ROS log severity level to configurable parameter in launch file
- Multicast support in API
- VC2/VC3 support in FileReader/DeviceInterface

## v4.0.0
- ROS updates: 
	1. Automatically read FPS and pixel number from device/recording
	2. Change path of json/recording to a relative path in the launch file
	3. Change "frame_id" parameter to configurable parameter
- PRO EOL: pro device/recordings end of support 

## v3.5.0
- ros updates: multiple reflections and summation vizualition support.

## v3.4.4
- ignore parsing wrong/old OC/DC TLVs

## v3.4.2
- pcp48K tlv handler fix

## v3.4.1
- summation direction calculation fix
- handling tlv 0x41 disorder

## v3.4.0
- adding TLV pcp48K
- adding TLV's OC/DC
- demo program distribution
- adding false positive alarm to ros
- distance offset rviz fix

## v3.3.4
- raw writer indexing fix (unordered frames)
- initialize frame grabber sync fix
- taps handler sync fix(register/unregister taps)
- python wrapper - check_pixel_validity default value update (false)
- summation direction calculation fix 
- handling partial acp header (Rx/uart)
- csahrp wrapper DeviceMeta static struct update

## v3.2.2
- invz4_5 format support

## v3.1.1
- segmentation fault fix
- VC1 Galapagos issue
- PCPlus meta data grab type in innopy
- fw_version_hash report in connect fail

## v3.1.0
- get_dp issue in socketReceiveCallback fix

## v3.0.20
- tap test fix

## v3.0.19
-bug fix file reader memory leak
-blockage indexation fix file writer

## v3.0.18(alpha version)
- bug fix in device close(different solution for windows and linux)
- bug fix in blockage writer/reader
- support different login levels
- support all test in windows and linux( known issue in taps related tests)
- support demo program phase 1 

## v3.0.15(alpha version)
- memory leak fix in file reader
- known issue in blockage writer/reader - should not be used
- known issue in device interface - should not be used

## v3.0.14
-confidence negative range bug fix
-known issue in reading pcp data with file reader 

## v3.0.13
-summation pixel ghost aligned to native pixel ghost bug fix

## v3.0.12
-data per channel bug fix.

## v3.0.11
-exposing all pcl data in taps and get frame (including ghost and short range).

## v3.0.10
-expose ghost data in get frame

## v3.0.9
- confidence value bug fix: On INVZ4_3 (fw version prior to 0_7_11), Confidence value (and pfa respectively) no longer dependent on 2 bits reseved for grazing angle to be set to 0.
- added flush_queues option where starting a recording. when passing True all udp queues will be cleared before the recording starts

## v3.0.8
- check lidar version matches device
- split pc+ to high\low resolution(summation)
- added noise filter
- playback of pc+ recordings

## v3.0.7
- critical mem leaks fixes over Linux;
- various bugs fixes

## v3.0.6
- added get connection status
- added summation confidence
- added validity field on measurement struct
- added pfa field on measurement struct
- added new grab_types : 
	Validity buffer for measurement
	validity buffer for summation
- added taps with validity
## v3.0.5
- fix File Reader change grabbed data types bug
- added ability to set validation filter on\off dor FileReader (check_pixel_validity)

## v3.0.4
- GetFrame supports ThetaPhi Values - use GRAB_TYPE_THETA_PHI dataAttribute
- bit field support in NDArray
- fixed file stream closing issues. note: use "del [file_reader_object]" to ensure the stream is closed
- fix bug in summation and reflections parsing
- by default pixel validity will not be checked and all  pixels will show. set in lidar.json check_pixel_validity:True to filter invalid pixels

## v3.0.3
- fixed python issue: cannot import innopy.api on linux machine. 
- UDP communication changed to PCAP instead of UDP socket - Linux support
- python interface added supporting UdpReceiver - allow get raw packets from device.

## v3.0.1
- !!!!!!!!!!this api is supported for windows only!!!!!!!!!!
- UDP communication changed to PCAP instead of UDP socket- need to install WINPCAP from "S:\RnD\SW\Versions\unofficial\Studio\Player\WinPcap_4_1_3.exe"
- Added IRawWriter interface supporting INVZ4_3 writer, see IRawWriter.h
- Added IRawReader interface supporting INVZ4_3 INVZ2 reader, see IRawReader.h
- Innopy Write raw packet to file (INVZ4_3) implementation
- support invz 4.3/4.4
- create DeviceInterface without device Metadata (CM scenario)
- crc_type added to lidar json. 1 (default)= innoviz CRC, 2= autosar CRC

## v3.0.0
- tap event including data
- new get_frame interface. old get get_frame is obselete
	- in device for each data buffer you should activate before grabbing with ActivateBuffer
- python API:
	- need to pass array of FrameDataAttributes.Can be created only with GrabType enum
	- return result object instead of tuple
	- result:
		- frame_number
		- timestamp  (microseconds from epoch)
		- success- indicates if all buffers where successfully returned
		- results- dictionay of results according to requested buffers
		
=======
## v2.0.26
- fixed set_dp uin32 bug
## v2.0.25
- fixed python issue :support number types without _t in lidar_v1 json file
## v2.0.24
- fix crash that happens once in a while on unregister callback
## v2.0.23
- fixed bug invz2 incomplete last frame causing endless loop
## v2.0.22
- fixed get_frame with summation bug
- added log level for FileReader (default=3)
## v2.0.21
- added device_close to python wrapper
- python close interface if connect failed
## v2.0.20
- Update send_tlv() so it will be able to return error TLV using return_error_tlv bool argument
- Fixed bug of string parametes return
- Update get_frame() to return directions using calc_directions bool argument 
- Allow cnc timeout to be less tahn 1000 ms
## v2.0.19
- adding unix (ubunto 16.04) support

##v2.0.18
- reading w and di form .csv if found inside recording folder. csv precision must be atleast 6 digits.
- at this version, no unix support.
- FrameDataAttributes.json -> frame_data_to_grab -> "measurements", frame itemsize should be updated to 17.

## v2.0.17
- Measurments now return also x,y,z. (ie: distance, confidence, reflectivity, noise, x, y, z) 

## v2.0.16
- Added blockage taps

## v2.0.15
- FrameMeta added as one of the data collected when grabbing frame measurements style using innopy get_frame() 

## v2.0.14
- summation in taps and in point cloud

##v2.0.13
- fixed get datapoint by Id bug

##v2.0.12
- Fix recording bug cuased by buffer manipulation when parsing

##v2.0.11
- Change offsets in recording index file to be 64 bit 

##v2.0.10
- Fix innopy FileReader get_frame() with measurments for invz4

##v2.0.9
- auto taps from point cloud
- get_frame returns measurements

##v2.0.8
- Fix FileReader get_frame() stuck with invz2 files
- Add CRC required for read/write register request

##v2.0.7
- FileReader close all stream file on destruction
- Fix Callback triggered after unregister bug
- innopy DeviceInterface get_frame implementation

##v2.0.6
- Add guards on read/write tap buffers
- Remove demoProgram from distribution

##v2.0.5
- Change index file name suffix to '.indx2'

##v2.0.4
- added send_retries parameter in lidar.json to allow retries on communication timeout. default is 1
- added index file for invz2 files (.invz)

##v2.0.3
- prevent get for taps
- add log level to python in deviceinit

##v2.0.2
- Registers r\w by access

##v2.0.1
- Added UART support

##v2.0.0
-New API calls
	
	DeviceInterface:
	INVZ_API IDevice *DeviceInit(std::string config_filename, std::string api_log_filename = "", uint32_t api_log_severity_level = 3);
	INVZ_API void DeviceClose(IDevice* device_interface);
	/* IReader */
	Result GetFileFormat(uint32_t& file_format);
	Result GetPixelsPerFrame(size_t &ppf);
	Result GetDirections(vector3 *directions, size_t directions_len);
	Result GetTimings(uint32_t* timings, size_t timings_len);
	Result GetDeviceMetadata(DeviceMeta& device_meta);
	Result GetNumOfFrames(size_t &number_of_frames);
	Result GetPacket(byte *packet, size_t packet_max_size, size_t& packet_size, uint64_t& timestamp, uint16_t &port, uint32_t& channel_type);
	Result GetFrameDataAttributes(FrameDataAttributes* frame_data_attrs, size_t& frame_data_attrs_count);
	Result GrabFrame(FrameDataUserBuffer* frame_data_buffers, uint32_t frame_data_count, uint32_t& frame_number, uint32_t frame_index);
	/* ICnc */
	Result EstablishConnection();
	Result Disconnect();
	Result SendTLV(TlvPack &request, TlvPack &response);
	Result Connect(uint8_t &actual_level, uint8_t request_level, std::string password);
	Result IsConnected(bool &connected);
	Result Ping(uint32_t payload_size);
	Result GetParameterById(uint16_t parameter_id, size_t& parameter_len, uint8_t* parameter_value);
	Result GetParameterByName(std::string parameter_name, size_t& parameter_len, uint8_t* parameter_value);
	Result GetParameterByDataPoint(const DataPoint* parameter_dp, size_t& parameter_len, uint8_t* parameter_value);
	Result SetParameterById(uint16_t parameter_id, size_t parameter_len, uint8_t* parameter_value, bool burn_to_flash);
	Result SetParameterByName(std::string parameter_name, size_t parameter_len, uint8_t* parameter_value, bool burn_to_flash);
	Result SetParameterByDataPoint(const DataPoint* parameter_dp, size_t parameter_len, uint8_t* parameter_value, bool burn_to_flash);
	Result SetParameterTapStateById(uint16_t parameter_id, bool state);
	Result SetParameterTapStateByName(std::string parameter_name, bool state);
	Result SetParameterTapStateByDataPoint(const DataPoint* parameter_dp, bool state);
	Result BurnParameterCurrentValueById(uint16_t parameter_id);
	Result BurnParameterCurrentValueByName(std::string parameter_name);
	Result BurnParameterCurrentValueByDataPoint(const DataPoint* parameter_dp);
	Result ReadRegisterByAddress(uint32_t absolute_address, uint32_t width, uint32_t bit_offset, uint32_t& register_value);
	Result ReadRegisterByName(std::string register_name, uint32_t& register_value);
	Result ReadRegister(const Register* register_struct, uint32_t& register_value);
	Result WriteRegisterByAddress(uint32_t absolute_address, uint32_t width, uint32_t bit_offset, uint32_t register_value);
	Result WriteRegisterByName(std::string register_name, uint32_t register_value);
	Result WriteRegister(const Register* register_struct, uint32_t register_value);
	Result GetNorPartitionSize(uint32_t partition_id, size_t& partition_length);
	Result ReadNorPartition(uint32_t partition_id, uint32_t& buffer_length, uint8_t* buffer);
	Result WriteNorPartition(uint32_t partition_id, uint32_t buffer_length, uint8_t* buffer);
	Result EraseNorPartition(uint32_t partition_id);
	Result GetDataPointById(uint16_t parameter_id, const DataPoint*& data_point);
	Result GetDataPointByName(std::string parameter_name, const DataPoint*& data_point);
	Result GetDataPointByIndex(uint32_t parameter_index, const DataPoint*& data_point);
	Result GetDataPointCount(uint32_t& parameter_count);
	Result GetRegisterByName(std::string register_name, const Register*& register_struct);
	Result GetRegisterByIndex(uint32_t register_index, const Register*& register_struct);
	Result GetRegisterCount(uint32_t& register_count);
	/* Recording */
	Result StartRecording(std::string file_path);
	Result StopRecording();
	Result Record(double seconds, std::string file_path = "");
	/* Callbacks */
	Result RegisterLogMessageCallback(std::function<void(DeviceLogMsg)> callback);
	Result UnregisterLogMessageCallback();
	Result RegisterTapCallback(std::function<void(TapHandler)> callback);
	Result UnregisterTapCallback();
	Result RegisterTlvCallback(std::function<void(TlvPack)> callback);
	Result UnregisterTlvCallback();
	Result RegisterFrameCallback(std::function<void(uint32_t)> callback);
	Result UnregisterFrameCallback();

-Changed
	- Connect()
	- GetByteArray - can be achived using  
	- SetByteArray
	- SetStrValue
	- SetActivationState
	- *GetStringValue
	- *GetDataPointsList
	- FindDataPointWithName 
	- FindDataPointWithId
	- GetDataPointByIndex
	- GetRegisterByIndex
	- IsConnected
	- *GetPixelSummationsPerFrame
	- *GetPartitionLength
	- *ReceivePartition
	- *ErasePartition
	- *SetBypassMode


-Removed: ( Add new funcionality/alternative )
	- ResetLidar - can be achived by using SetParameterByName
	- RunCalibration - Not supported any more 
	- GetLidarName - can be achived by using GetParameterByName
	- GetSerialNumber - can be achived by using GetParameterByName
	- GetVersion - can be achived by using GetParameterByName
	- GetMacAddress - can be achived by using GetParameterByName
	- GetFrame - look at GrabFrame
	- GetSummationFrame - look at GrabFrame
	- GetTrackingFrame - look at GrabFrame
	- GetObjectsFrame - look at GrabFrame
	- GetPCPlusFrame - look at GrabFrame
	- GetLandmarksFrame - look at GrabFrame
	- GetPolesFrame - look at GrabFrame
	- GetBlockageClassification - look at GrabFrame
	- GetValuesByteArray - no alternative
	- IsGPSSync - can be achived by using GetParameterByName
	- UpgradeFirmware - Not supported any more 
	- ReceiveParametersFile - Not supported any more 
	- LoadParametersFile - Not supported any more 
	- SendFile - Not supported any more 
	- ReceiveFile - Not supported any more 
	- ListDir - Not supported any more 
	- GetDir - Not supported any more 
	- EraseFile - Not supported any more 
	- GetLidarState - Not supported any more 
	- SetLidarState - Not supported any more 
	- AddVirtualChannel - Not supported any more 


## v1.3.0
- inzv2 recording support in pro and invz3 recording support for om
- python_wrapper -> FileReader -> get_packet returns 4 values, packet is numpy array in packet size length.
- windows distrubution create debug and releas libs

### Internal
* changes
	- file reader assumes invz2 if source is file and not directory
	- file reader object indexing removed
* bugs:
	- disconnect without connect caused crash since pointers where missing default assignment.

## v1.2.14
- linux support bug fix

### Internal
- added namespace for FileWriter and INVZ Writer
- integrated python_wrapper project into main solution/cmake
- added watcher to constantly copy .dll required by python_wrapper to innopy folder for development
- Connect device using profile file is supported by innopy
	- Find the following configuration files in the python examples folder:
		- CmParam.json - for connecting the compute module
		- OmDirectParam.json - for connecting the optical module directly
		- OmBypassParam.json - for connecting the optical module through the compute module (bypass mode)
		- ProParam.json - for connecting the Pro lidar
	- Example for connection:
		- di = DeviceInterface(profile_file_path='OmDirectParam.json')
- Receive and load parameters partition funcionality added
- CnC max packet size configurable
- Blockage and Weather received by GetFrame() are not necessarily belong to the asked frame since they received by different port
- When recording is enabled, the new format of invz is recorded and can be read packet-by-packet using the FileReader
* bugs:
	- innvoiz api cppsharp wrapper generation fails 

## v1.2.13
- Adding send/receive/erase file ability to python wrapper

## v1.2.12
- when calling SetByteArray with set_param=true first a regular set is performed and then burn current value is called
- fix crash when reserved part in TLV is not zero

## v1.2.11
- changed to ACP2.0

## v1.2.10
- parsing registers json will consider sub_address value
- register address will be in hex in log

## v1.2.9
- send ping on indirect get timeout to patch LiDAR's udp bug
- fix GetFrame with objects bug
- indirect get support multiple TLV

## v1.2.8
- added read block by block from file reader (FileReader::GetBlock(byte *block, size_t block_max_size, size_t &bytes_read))
- get_all_dp_details bug fix
- Add 5 seconds timeout to packet dequeue so get_frame will not be stuck
- Add frame number to blockage classification (and all other frame oriented BlockTlv)
- Log handleing added to ACP supported device

## v1.2.7
- exposed do_handshake to deviceInit and python wrapper

## v1.2.6
- change getframe to fail when part of the frame is missing
- Add new funcionality of read/write register from device (only for dev user)

## v1.2.5
- Added blockage classification(GetBlockageClassification) and environmental blockage in framemetadata
- Support padding(4 bytes aligned) in pro PC measurments block
- Buf fix - '/' instead of '\' when save parameters using API

## v1.2.4
- New sensor pose structure in get detailed frame. also now active

## v1.2.3
- Added Weather + blockage in get frame and detailed frame
- Device init from json profile file for example:
```
	{
		"ip":"10.1.1.112",
		"cnc_port":7000,
		"data_port":8200,
		"dlt_port":8201,
		"device_type":2,
		"pc_version":0,
		"pc_port":0,
		"packet_buffer_size":300,
		"do_handshake": true
	}
```

## v1.2.2
- Bug fix - Last measurement of every block was missing

## v1.2.1
- GetDetailedFrame splitted to get trackedobjects or objects
- Added SendTLV to send and get TLV messgaes (for QA purposes)	
- Added NewTLV callback to get notified when a new tlv has arrived. the callback sends TlvPack structure

## v1.2.0
- support both PRO and ACP protocol
- Changed api expections to Result struct containing error code and description
- added GetDetailedFrame to get a full frame +object detection+ sensor pose
	- if meas_size==0 then the point cloud data will be skipped
	- the detailed frame will return the first frame number match for all parts
- init device function cahanged includes the following parameters:
	- ip
	- cnc_port - port for control interface. if 0 no control will be available and directions must be read from a file located in the running folder.
		supported files:
		- directions.bin: binary file with 4 first bytes for num of directions. after that x,z,y floats for each directions
		- directions.csv: csv file with x,y,z rows for each direction
	- data_port - port for udp data ( point cloud, objects, etc)
	- dlt_port - port for udp debug
	- patch_pcl_port - patch port to get point cloud from other port than the data port (will be removed in the future)
	- device_type - device type : PRO, Compute module(ACP)
	- patch_pcl_version - patch point cloud version to support mixed PRO and ACP protocls (will be removed in the future)
- newframe callback will notify if the new frame is only point cloud or includes objects
	- if object frame use GetDetailedFrame to get all data

## v1.1.2
- bug fix with record

### python wrapper
- bug fix with exposing start_record, end_record


## v1.1.1
### python wrapper
- exposed set_tap_activation_state
- expose start_record, end_record

### CS Wrapper
first release, internal


## v1.1.0
- documentation (innnovizApi.chm) separates technician functions from others.
- callbacks added on taps and device logs (technician mode)
- report callbacks added to several functions.
- .invz file "index" mode now autogenerated .indx file for faster file read.
- misc bug fix.
- API logs initializer was exposed (usage is optional).  
  this could be used to change logs level from 'info' to 'trace' for more detailed debug information.

### python wrapper
- exposed all added features
- exposed disconnect \ connect functions. Connect still occurs on DeviceInterface constructor.

### ros api
steaming ros PointCloud2 from file or device

