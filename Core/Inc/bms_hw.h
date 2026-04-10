//bms_hw.h
#ifndef BMS_HW_H_
#define BMS_HW_H_

#include <stdbool.h>
#include <stdint.h>

void BMS_HW_Init(void);

void BMS_HW_SetChargeEnable(bool on);
void BMS_HW_SetDischargeEnable(bool on);


bool BMS_HW_UserButtonPressed(void);

#endif /* BMS_HW_H_ */

