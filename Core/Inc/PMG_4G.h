/*
 * PMG_4G.h
 *
 *  Created on: Aug 12, 2022
 *      Author: devink
 */

#ifndef INC_PMG_4G_H_
#define INC_PMG_4G_H_

#include <stdint.h>

typedef enum
{
	ePMG_4G_Waiting_For_Network_Connection = 0,
	ePMG_4G_ALert_Controller_Network_Is_Online,
	ePMG_4G_Idle,
	ePMG_4G_Sending_Production_Test_Ping,
	ePMG_4G_Receiving_Production_Test_Ping_Response,
	ePMG_4G_Received_Production_Test_Ping_Response,
	ePMG_4G_Turning_Off_Radio,
	ePMG_4G_Off
}PMG_4G_States;

typedef struct
{
	PMG_4G_States Current_State;

	uint32_t Task_ID;
	uint32_t Ping_Task_ID;
	uint32_t TFTP_Ping_Task_ID;
}PMG_4G;

void Init_PMG_4G(void);

#endif /* INC_PMG_4G_H_ */
