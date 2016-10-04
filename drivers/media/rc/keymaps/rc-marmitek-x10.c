/*
 * Marmitek Driver
 */

#include <linux/module.h>
#include <media/rc-map.h>

static struct rc_map_table marmitek_x10[] = {
	{ 0xf00f, KEY_S }, // Power
	{ 0xd22d, KEY_LEFT },
	{ 0xd12e, KEY_RIGHT },
	{ 0xd52a, KEY_UP },
	{ 0xd32c, KEY_DOWN },	
	{ 0x52ad, KEY_KPENTER },
	{ 0xa05f, KEY_F8 }, // Mute
	{ 0xe01f, KEY_F9 }, // Vol-
	{ 0x609f, KEY_F10 }, // Vol+
	{ 0xc03f, KEY_DOWN }, // Channel-
	{ 0x40bf, KEY_UP }, // Channel+
	{ 0x827d, KEY_KP1 },
	{ 0x42bd, KEY_KP2 },
	{ 0xc23d, KEY_KP3 },
	{ 0x22dd, KEY_KP4 },
	{ 0xa25d, KEY_KP5 },
	{ 0x629d, KEY_KP6 },
	{ 0xe21d, KEY_KP7 },
	{ 0x12ed, KEY_KP8 },
	{ 0x926d, KEY_KP9 },
	{ 0x02fd, KEY_KP0 },
	{ 0xb649, KEY_SEMICOLON }, // Menu
	{ 0xc936, KEY_ESC }, // Exit
	{ 0x3ac5, KEY_F2 }, // Skip-
	{ 0xd827, KEY_F3 }, // Skip+
	{ 0xba45, KEY_F4 }, // AV
	{ 0xff00, KEY_K }, // Record
	{ 0xb04f, KEY_P }, // Play
	{ 0x38c7, KEY_R }, // Rewind
	{ 0xb847, KEY_F }, // Forward
	{ 0x708f, KEY_X }, // Stop
	{ 0x728d, KEY_SPACE }, // Pause
};

static struct rc_map_list marmitek_x10_map = {
	.map = {
		.scan    = marmitek_x10,
		.size    = ARRAY_SIZE(marmitek_x10),
		.rc_type = RC_TYPE_OTHER,
		.name    = RC_MAP_MARMITEK_X10,
	}
};

static int __init init_rc_map_marmitek_x10(void)
{
	return rc_map_register(&marmitek_x10_map);
}

static void __exit exit_rc_map_marmitek_x10(void)
{
	rc_map_unregister(&marmitek_x10_map);
}

module_init(init_rc_map_marmitek_x10);
module_exit(exit_rc_map_marmitek_x10);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Remi Pachon <remi.pachon@gmail.com>");
