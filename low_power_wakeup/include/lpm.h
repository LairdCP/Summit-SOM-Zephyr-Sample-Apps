/*
 * Copyright 2020 NXP
 * Copyright 2022, Laird Connectivity
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _LPM_H_
#define _LPM_H_

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function is used to increase the count of the block event.
 */
void lpm_increase_block_sleep_count(void);
/*!
 * @brief This function is used to decrease the count of the block event.
 */
void lpm_decrease_block_sleep_count(void);
/*!
 * @brief This function is used to judge if the system could enter low power mode.
 * @return Return true if there is no block event exists.
 */
bool lpm_allow_sleep(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/
#endif /* _LPM_H_ */
