#ifndef UNIFIED_CONFIG_H
#define UNIFIED_CONFIG_H


/* Enable bytes for HW modules */
#define MODULE_MS5837_ENABLE		0b00000001
#define MODULE_DVL_ENABLE			0b00000010
#define MODULE_EMBEDSYS_ENABLE		0b00000100
#define MODULE_PWRSYS_ENABLE		0b00001000
#define MODULE_BRLIGHT_ENABLE		0b00010000
#define MODULE_WATCHDOG_ENABLE		0b00100000

/* Global settings */
namespace GlobalSettings
{
	const static char can_bus_interface[] = "vcan0";
	static uint8_t module_enabled_field = 0xFF;
	static bool do_device_polling = true;

	/* embedded system interaction */
	const static uint8_t dreq_id = 0x020;
	const static uint8_t dres_id = 0x021;
	const static uint8_t stow_id = 0x022;
}
#endif
