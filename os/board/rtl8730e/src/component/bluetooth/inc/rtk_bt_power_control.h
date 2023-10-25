/**
 * @file      rtk_bt_power_control.h
 * @author
 * @brief     Bluetooth Common function definition
 * @copyright Copyright (c) 2022. Realtek Semiconductor Corporation. All rights reserved.
 */

#ifndef __RTK_BT_POWER_CONTROL_H__
#define __RTK_BT_POWER_CONTROL_H__

#include <rtk_bt_def.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef void (*rtk_bt_ps_callback)(void);

/**
 * @defgroup  bt_power_control BT power control APIs
 * @brief     BT power control APIs
 * @ingroup   BT_APIs
 * @{
 */

/**
* @brief     bt enable power save.
* @param     None
* @return    None
*/
void rtk_bt_enable_power_save(void);

/**
* @brief     bt disable power save.
* @param     None
* @return    None
*/
void rtk_bt_disable_power_save(void);

/**
* @brief     bt power save init.
* @param     None
* @return    None
*/
void rtk_bt_power_save_init(void);

/**
* @brief     bt power save deinit.
* @param     None
* @return    None
*/
void rtk_bt_power_save_deinit(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __RTK_BT_POWER_CONTROL_H__ */
