/*
 * Copyright (C) 2014  INRIA.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell_commands
 * @{
 *
 * @file
 * @brief       Provides prototypes and sets up available shell commands
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Zakaria Kasmi <zkasmi@inf.fu-berlin.de>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * @}
 */

#include <stdlib.h>
#include "shell_commands.h"

extern int _reboot_handler(int argc, char **argv);

#if( defined(MODULE_CONFIG) && defined(MODULE_CONFIG_SHELL_ENABLE) )
extern int _id_handler(int argc, char **argv);
#endif

#if( defined(MODULE_LPC_COMMON) && defined(MODULE_LPC_COMMON_SHELL_ENABLE) )
extern int _heap_handler(int argc, char **argv);
#endif

#if( defined(MODULE_PS) && defined(MODULE_PS_SHELL_ENABLE) )
extern int _ps_handler(int argc, char **argv);
#endif

#if( defined(MODULE_SHT11) && defined(MODULE_SHT11_SHELL_ENABLE) )
extern int _get_temperature_handler(int argc, char **argv);
extern int _get_humidity_handler(int argc, char **argv);
extern int _get_weather_handler(int argc, char **argv);
extern int _set_offset_handler(int argc, char **argv);
#endif

#if( defined(MODULE_LTC4150) && defined(MODULE_LTC4150_SHELL_ENABLE) )
extern int _get_current_handler(int argc, char **argv);
extern int _reset_current_handler(int argc, char **argv);
#endif

#if( defined(MODULE_AT30TSE75X) && defined(MODULE_AT30TSE75X_SHELL_ENABLE) )
extern int _at30tse75x_handler(int argc, char **argv);
#endif

#if( defined(MODULE_SAUL_REG) && defined(MODULE_SAUL_REG_SHELL_ENABLE) )
extern int _saul(int argc, char **argv);
#endif

#if( defined(MODULE_PERIPH_RTC) && defined(MODULE_PERIPH_RTC_SHELL_ENABLE) )
extern int _rtc_handler(int argc, char **argv);
#endif

#if( defined(MODULE_MCI) && defined(MODULE_MCI_SHELL_ENABLE) )
extern int _get_sectorsize(int argc, char **argv);
extern int _get_blocksize(int argc, char **argv);
extern int _get_sectorcount(int argc, char **argv);
extern int _read_sector(int argc, char **argv);
extern int _read_bytes(int argc, char **argv);
#endif

#if( defined(MODULE_GNRC_ICMPV6_ECHO) && defined(MODULE_GNRC_ICMPV6_ECHO_SHELL_ENABLE) )
#if defined(MODULE_XTIMER)
extern int _icmpv6_ping(int argc, char **argv);
#else
    #error MODULE_GNRC_ICMPV6_ECHO_SHELL_ENABLE does not work without MODULE_XTIMER
#endif
#endif

#if( defined(MODULE_RANDOM) && defined(MODULE_RANDOM_SHELL_ENABLE) )
extern int _random_init(int argc, char **argv);
extern int _random_get(int argc, char **argv);
#endif

#if( defined(MODULE_GNRC_IPV6_NIB) && defined(MODULE_GNRC_IPV6_NIB_SHELL_ENABLE) )
extern int _gnrc_ipv6_nib(int argc, char **argv);
#endif

#if( defined(MODULE_GNRC_NETIF) && defined(MODULE_GNRC_NETIF_SHELL_ENABLE) )
extern int _gnrc_netif_config(int argc, char **argv);
#if defined(MODULE_GNRC_TXTSND)
extern int _gnrc_netif_send(int argc, char **argv);
#endif
#endif

#if( defined(MODULE_FIB) && defined(MODULE_FIB_SHELL_ENABLE) )
extern int _fib_route_handler(int argc, char **argv);
#endif

#if( defined(MODULE_GNRC_IPV6_NC) && defined(MODULE_GNRC_IPV6_NC_SHELL_ENABLE) )
extern int _ipv6_nc_manage(int argc, char **argv);
extern int _ipv6_nc_routers(int argc, char **argv);
#endif

#if( defined(MODULE_GNRC_IPV6_WHITELIST) && defined(MODULE_GNRC_IPV6_WHITELIST_SHELL_ENABLE) )
extern int _whitelist(int argc, char **argv);
#endif

#if( defined(MODULE_GNRC_IPV6_BLACKLIST) && defined(MODULE_GNRC_IPV6_BLACKLIST_SHELL_ENABLE) )
extern int _blacklist(int argc, char **argv);
#endif

#if( defined(MODULE_GNRC_RPL) && defined(MODULE_GNRC_RPL_SHELL_ENABLE) )
extern int _gnrc_rpl(int argc, char **argv);
#endif

#if( defined(MODULE_GNRC_SIXLOWPAN_CTX) && defined(MODULE_GNRC_SIXLOWPAN_CTX_SHELL_ENABLE) )
#if defined(MODULE_GNRC_SIXLOWPAN_ND_BORDER_ROUTER)
extern int _gnrc_6ctx(int argc, char **argv);
#endif
#endif

#if( defined(MODULE_CCN_LITE_UTILS) && defined(MODULE_CCN_LITE_UTILS_SHELL_ENABLE) )
extern int _ccnl_open(int argc, char **argv);
extern int _ccnl_content(int argc, char **argv);
extern int _ccnl_interest(int argc, char **argv);
extern int _ccnl_fib(int argc, char **argv);
#endif

#if( defined(MODULE_SNTP) && defined(MODULE_SNTP_SHELL_ENABLE) )
extern int _ntpdate(int argc, char **argv);
#endif

#if( defined(MODULE_VFS) && defined(MODULE_VFS_SHELL_ENABLE) )
extern int _vfs_handler(int argc, char **argv);
extern int _ls_handler(int argc, char **argv);
#endif

#if( defined(MODULE_CONN_CAN) && defined(MODULE_CONN_CAN_SHELL_ENABLE) )
extern int _can_handler(int argc, char **argv);
#endif

const shell_command_t _shell_command_list[] = {
    {"reboot", "Reboot the node", _reboot_handler},
#if( defined(MODULE_CONFIG) && defined(MODULE_CONFIG_SHELL_ENABLE) )
    {"id", "Gets or sets the node's id.", _id_handler},
#endif
#if( defined(MODULE_LPC_COMMON) && defined(MODULE_LPC_COMMON_SHELL_ENABLE) )
    {"heap", "Shows the heap state for the LPC2387 on the command shell.", _heap_handler},
#endif
#if( defined(MODULE_PS) && defined(MODULE_PS_SHELL_ENABLE) )
    {"ps", "Prints information about running threads.", _ps_handler},
#endif
#if( defined(MODULE_SHT11) && defined(MODULE_SHT11_SHELL_ENABLE) )
    {"temp", "Prints measured temperature.", _get_temperature_handler},
    {"hum", "Prints measured humidity.", _get_humidity_handler},
    {"weather", "Prints measured humidity and temperature.", _get_weather_handler},
    {"offset", "Set temperature offset.", _set_offset_handler},
#endif
#if( defined(MODULE_LTC4150) && defined(MODULE_LTC4150_SHELL_ENABLE) )
    {"cur", "Prints current and average power consumption.", _get_current_handler},
    {"rstcur", "Resets coulomb counter.", _reset_current_handler},
#endif
#if( defined(MODULE_AT30TSE75X) && defined(MODULE_AT30TSE75X_SHELL_ENABLE) )
    {"at30tse75x", "Test AT30TSE75X temperature sensor", _at30tse75x_handler},
#endif
#if( defined(MODULE_MCI) && defined(MODULE_MCI_SHELL_ENABLE) )
    {DISK_READ_SECTOR_CMD, "Reads the specified sector of inserted memory card", _read_sector},
    {DISK_READ_BYTES_CMD, "Reads the specified bytes from inserted memory card", _read_bytes},
    {DISK_GET_SECTOR_SIZE, "Get the sector size of inserted memory card", _get_sectorsize},
    {DISK_GET_SECTOR_COUNT, "Get the sector count of inserted memory card", _get_sectorcount},
    {DISK_GET_BLOCK_SIZE, "Get the block size of inserted memory card", _get_blocksize},
#endif
#if( defined(MODULE_GNRC_ICMPV6_ECHO) && defined(MODULE_GNRC_ICMPV6_ECHO_SHELL_ENABLE) )
#if defined(MODULE_XTIMER) 
    { "ping6", "Ping via ICMPv6", _icmpv6_ping },
#endif
#endif
#if( defined(MODULE_RANDOM) && defined(MODULE_RANDOM_SHELL_ENABLE) )
    { "random_init", "initializes the PRNG", _random_init },
    { "random_get", "returns 32 bit of pseudo randomness", _random_get },
#endif
#if( defined(MODULE_PERIPH_RTC) && defined(MODULE_PERIPH_RTC_SHELL_ENABLE) )
    {"rtc", "control RTC peripheral interface",  _rtc_handler},
#endif
#if( defined(MODULE_GNRC_IPV6_NIB) && defined(MODULE_GNRC_IPV6_NIB_SHELL_ENABLE) )
    {"nib", "Configure neighbor information base", _gnrc_ipv6_nib},
#endif
#if( defined(MODULE_GNRC_NETIF) && defined(MODULE_GNRC_NETIF_SHELL_ENABLE) )
    {"ifconfig", "Configure network interfaces", _gnrc_netif_config},
#if( defined(MODULE_GNRC_TXTSND) && defined(MODULE_GNRC_TXTSND_SHELL_ENABLE) )
    {"txtsnd", "Sends a custom string as is over the link layer", _gnrc_netif_send },
#endif
#endif
#if( defined(MODULE_FIB) && defined(MODULE_FIB_SHELL_ENABLE) )
    {"fibroute", "Manipulate the FIB (info: 'fibroute [add|del]')", _fib_route_handler},
#endif
#if( defined(MODULE_GNRC_IPV6_WHITELIST) && defined(MODULE_GNRC_IPV6_WHITELIST_SHELL_ENABLE) )
    {"whitelist", "whitelists an address for receival ('whitelist [add|del|help]')", _whitelist },
#endif
#if( defined(MODULE_GNRC_IPV6_BLACKLIST) && defined(MODULE_GNRC_IPV6_BLACKLIST_SHELL_ENABLE) )
    {"blacklist", "blacklists an address for receival ('blacklist [add|del|help]')", _blacklist },
#endif
#if( defined(MODULE_GNRC_RPL) && defined(MODULE_GNRC_RPL_SHELL_ENABLE) )
    {"rpl", "rpl configuration tool ('rpl help' for more information)", _gnrc_rpl },
#endif
#if( defined(MODULE_GNRC_SIXLOWPAN_CTX) && defined(MODULE_GNRC_SIXLOWPAN_CTX_SHELL_ENABLE) )
#if defined(MODULE_GNRC_SIXLOWPAN_ND_BORDER_ROUTER)
    {"6ctx", "6LoWPAN context configuration tool", _gnrc_6ctx },
#endif
#endif
#if( defined(MODULE_SAUL_REG) && defined(MODULE_SAUL_REG_SHELL_ENABLE) )
    {"saul", "interact with sensors and actuators using SAUL", _saul },
#endif
#if( defined(MODULE_CCN_LITE_UTILS) && defined(MODULE_CCN_LITE_UTILS_SHELL_ENABLE) )
    { "ccnl_open", "opens an interface or socket", _ccnl_open },
    { "ccnl_int", "sends an interest", _ccnl_interest },
    { "ccnl_cont", "create content and populated it", _ccnl_content },
    { "ccnl_fib", "shows or modifies the CCN-Lite FIB", _ccnl_fib },
#endif
#if( defined(MODULE_SNTP) && defined(MODULE_SNTP_SHELL_ENABLE) )
    { "ntpdate", "synchronizes with a remote time server", _ntpdate },
#endif
#if( defined(MODULE_VFS) && defined(MODULE_VFS_SHELL_ENABLE) )
    {"vfs", "virtual file system operations", _vfs_handler},
    {"ls", "list files", _ls_handler},
#endif
#if( defined(MODULE_CONN_CAN) && defined(MODULE_CONN_CAN_SHELL_ENABLE) )
    {"can", "CAN commands", _can_handler},
#endif
    {NULL, NULL, NULL}
};
