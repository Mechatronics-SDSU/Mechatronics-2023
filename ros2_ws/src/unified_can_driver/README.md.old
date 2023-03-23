# Unified CAN Driver notes
## (There is currently no official documentation, however this may provide some insight into how the driver works)
********************************************************************************
```
	Hierarchy
		base_driver				always start
		can_send_service		always start
		command_send_service	always start
		module_loader			always start
			ms5837			conditional		00000001	(Subject to change)
			dvl				conditional		00000010	(Subject to change)
			embedsys		conditional		00000100	(Subject to change)
			pwrsys			conditional		00001000	(Subject to change)
			brlight			conditional		00010000 	(Subject to change)
			brping1			conditional		00100000	(Subject to change)
			...
```
EX: ros2 run unified_can_driver unified_can_driver --ros-args [ arguements (see below) ]
 - `-p module_bitfield:=[bitfield]` Used to determine which modules to load when the driver starts 
 - `-p do_module_polling:=[true|false]` Disables continous CAN polling
 - `-p can_bus_interface:=interface]` Specify the interface
 
********************************************************************************

### TODO:	
- [x] IMPLEMENT DEFINED MBOX_INTERFACE***
- [x] RESTRUCTURE HANDLERS -> HANDLER TYPE SUBCLASS
- [x] ADD GLOBAL CONFIG/VARS FILE
- [x] DVL & Pressure initiation stage
- [x] Implement master can macro header
- [x] Implement formatted CanSend service
- [x] Restructure to Module based format
- [X] Implement Command-line arguments launch
- [X] Refactor to use Macro header
- [X] Delete unused/old files
- [ ] Rename services to better match function

********************************************************************************
### Module Template:
```
	Methods:
		mod_init:					whatever initialization the hardware needs
		dres_handle:				handles addressed Dres frames, outputs them to ros2 topics
		timer_callback:				whatever code should execute at a certain interval
		dres_info:					Whatever info/enable data received from embedded
	Variables:
		node_context				pointer to parent node
		mailbox_ptr					pointer to mbox_can
		module_enable_ti 			flag to bind timer_callback to 
		module_timer_len			interval to bind callback on
		module_enable_dres 			flag to enable dres handling
		module_device_id 			static int to hold device id
		module_topic_ct 	 		static int to hold topic count
		module_topic_ptr_array		array of topic pointers (jump table)
		module_hw_info				uint32_t containing received info/status from device
	Constructor
		node_context
		mailbox_ptr
```
********************************************************************************
### GlobalSettings [Defined in include/config.hpp and set by --ros-args]
	can_bus_interface 		: String, holds	Interface name
	do_device_polling		: Boolean, determines whether modules should poll the embedded system
	module_enable_field		: uint16_t, Not to be modified outside of main() function
