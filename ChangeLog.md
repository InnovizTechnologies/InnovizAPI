# Innoviz API Change Log

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
