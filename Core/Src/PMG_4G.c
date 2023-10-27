/*
 * PMG_4G.c
 *
 *  Created on: Aug 12, 2022
 *      Author: devink
 */

#include "PMG_4G.h"
#include "main.h"
#include "../../Nimbelink/QBG96/Nimbelink_QBG96.h"
#include "Scheduler.h"
#include "../../Drivers/ACI/Inc/Module_Bus.h"
#include "../../Drivers/ACI/Inc/LED.h"
#include "IP_Packet.h"
#include "DIGI_4G.h"
#include <string.h>

PMG_4G _4G;
extern Nimbelink_QBG96 * Radio;
//extern void pmg_print(const char *fmt, ...);
static void PMG_4G_Tasks(void * Task_Data);
static void PMG_4G_Send_Keep_Alive(void * Task_Data);
static void PMG_4G_Send_TFTP_Keep_Alive(void * Task_Data);
void Send_Online_Notification(void);
void Send_TFTP_Packet_To_Controller(uint32_t Src_Addr, uint16_t Src_Port, uint32_t Dst_Addr, uint16_t Dst_Port, uint8_t Protocol, uint8_t *Data, uint16_t Data_Size);

void Init_PMG_4G(void)
{
	_4G.Current_State = ePMG_4G_Waiting_For_Network_Connection;

	_4G.Task_ID = Start_Task(PMG_4G_Tasks, NULL, 100);
	_4G.Ping_Task_ID = Start_Task(PMG_4G_Send_Keep_Alive, NULL, 60000);
	_4G.TFTP_Ping_Task_ID = Start_Task(PMG_4G_Send_TFTP_Keep_Alive, NULL, 20000);
}

static void PMG_4G_Tasks(void * Task_Data)
{
	switch(_4G.Current_State)
	{
	case ePMG_4G_Waiting_For_Network_Connection:
	{
		if(Radio->Current_State == eNimbelink_QBG96_Online)
		{
			if(HAL_GPIO_ReadPin(nTEST_GPIO_Port, nTEST_Pin) == GPIO_PIN_RESET)
			{
				// We are in a production test environment
				Halt_Task(_4G.Ping_Task_ID);
				_4G.Current_State = ePMG_4G_Sending_Production_Test_Ping;
			}
			else
			{
				// We are in a normal running environment
				_4G.Current_State = ePMG_4G_ALert_Controller_Network_Is_Online;
			}
		}
	}
		break;
	case ePMG_4G_ALert_Controller_Network_Is_Online:
	{
		Send_Online_Notification();
//		Set_LED_On();
		_4G.Current_State = ePMG_4G_Idle;
	}
		break;
	case ePMG_4G_Idle:
	{
		if(Radio->Current_State == eNimbelink_QBG96_Off)
			_4G.Current_State = ePMG_4G_Off;
	}
		break;
	case ePMG_4G_Sending_Production_Test_Ping:
	{
		static int ping_trys = 1;

		if(ping_trys == 1)
			printf(">>>>>---- Send Manufacturing Test Ping --------->>>>\r\n");
		else if(ping_trys > 1 && ping_trys < 10)
			printf(">>>>>---- Send Manufacturing Test Ping %d --------->>>>\r\n", ping_trys);
		else
		{
			// We have tried 10 times with no response, fail the test
			printf("\r\n                     <<<<<<<    Manufacturing Test FAILED        >>>>>>\r\n");
			printf("\r\n");
			printf("                     FFFFF        AA       IIIIII    LL         !!\r\n");
			printf("                     FF          AAAA        II      LL         !!\r\n");
			printf("                     FF         AA  AA       II      LL         !!\r\n");
			printf("                     FFFFF      AA  AA       II      LL         !!\r\n");
			printf("                     FF        AAAAAAAA      II      LL         !!\r\n");
			printf("                     FF       AA      AA     II      LL         !!\r\n");
			printf("                     FF       AA      AA   IIIIII    LLLLLL     !!\r\n\r\n");
			Set_LED_On();

			_4G.Current_State = ePMG_4G_Turning_Off_Radio;
			return;
		}

		uint8_t ping = 'P';
		Nimbelink_QBG96_Send_Socket_Data(Radio, 0, &ping, 1);
		ping_trys++;

		Modify_Task_Timeout(_4G.Task_ID, 5000);						// Give the server 5 seconds to respond
		_4G.Current_State = ePMG_4G_Receiving_Production_Test_Ping_Response;
	}
		break;
	case ePMG_4G_Receiving_Production_Test_Ping_Response:
	{
		//
		_4G.Current_State = ePMG_4G_Sending_Production_Test_Ping;
	}
		break;
	case ePMG_4G_Received_Production_Test_Ping_Response:
	{
		printf("\r\n                     <<<<<<<    Manufacturing Test PASSED        >>>>>>\r\n");
		printf("\r\n");
		printf("                     PPPPPP       AA        SSSS    SSSS   EEEEE  DDDDD     !!\r\n");
		printf("                     PP   PP     AAAA      SS  SS  SS  SS  EE     DD  DD    !!\r\n");
		printf("                     PP   PP    AA  AA      SS      SS     EE     DD   DD   !!\r\n");
		printf("                     PPPPPP     AA  AA       SS      SS    EEEE   DD   DD   !!\r\n");
		printf("                     PP        AAAAAAAA       SS      SS   EE     DD   DD   !!\r\n");
		printf("                     PP       AA      AA   SS  SS  SS  SS  EE     DD  DD    !!\r\n");
		printf("                     PP       AA      AA    SSSS    SSSS   EEEEE  DDDDD     !!\r\n\r\n");
		Set_LED_On();

		_4G.Current_State = ePMG_4G_Turning_Off_Radio;
	}
		break;
	case ePMG_4G_Turning_Off_Radio:
	{
		printf("**************************************************************************\r\n");
		printf("**************************************************************************\r\n");
		printf("**                        MODEM SHUTTING DOWN                           **\r\n");
		printf("**     You may power down the module when SYS LED is extinguished.      **\r\n");
		printf("**            Note:  Module must be reset to restart modem!             **\r\n");
		printf("**************************************************************************\r\n");
		printf("**************************************************************************\r\n");

		Turn_Off_Nimbelink_QBG96(Radio);
		_4G.Current_State = ePMG_4G_Off;
		Radio->Current_State=eNimbelink_QBG96_Test;
		Set_LED_Off();
	}
		break;
	case ePMG_4G_Off:
	{
//		Set_LED_Off();
		if(Radio->Current_State == eNimbelink_QBG96_Online)
		{
				// We are in a normal running environment
				_4G.Current_State = ePMG_4G_ALert_Controller_Network_Is_Online;
		}
	}
		break;
	default:
		break;
	}
}

extern char *ping_get_super_ping(void);
static void PMG_4G_Send_Keep_Alive(void * Task_Data)
{
	unsigned char *sn;
	if(_4G.Current_State == ePMG_4G_Idle)
	{
		sn=(unsigned char *)ping_get_super_ping();
		if(sn==NULL)
		{
			uint8_t ping = 'P';
			Nimbelink_QBG96_Send_Socket_Data(Radio, 0, &ping, 1);
		}
		else
		{
			Nimbelink_QBG96_Send_Socket_Data(Radio, 0, sn, 18);
//			pmg_print("QBG96 send UDP: %s",sn);
		}
	}
}

static void PMG_4G_Send_TFTP_Keep_Alive(void * Task_Data)
{
	unsigned char *sn;
	if(_4G.Current_State == ePMG_4G_Idle)
	{
		sn=(unsigned char *)ping_get_super_ping();
		if(sn==NULL)
		{
			uint8_t ping = 'P';
			Nimbelink_QBG96_Send_Socket_Data(Radio, 1, &ping, 1);
		}
		else
		{
			Nimbelink_QBG96_Send_Socket_Data(Radio, 1, sn, 18);
//			pmg_print("QBG96 send TFTP: %s",sn);

		}
	}
}

void Send_Online_Notification(void)
{
//	dlog(DbgLvl_Info, "<<<<---[[[[ Send 4G Ready to Send Notifications ]]]]--->>>>\r\n");

	_4G_Modem_Info_Ex_t	rts;
	rts.WWAN_Flags = 0xFF;//wwan.Ready_Flags;
	rts.DigiFWVer = 0;//wwan.Digi_Version;
	strncpy((char*)rts.CCID, (char*)Radio->CCID, 20);
	strncpy((char*)rts.IMEI, (char*)Radio->IMEI, 16);
	for(int i = 0; i < 4; i++)
		rts.IP_Addr.octet[i] = Radio->Radio_IP[i];
	rts.Digi_State = 0;//wwan.Digi_State;
	rts.AC_Socket_Connect_State = 0;//wwan.AC_Socket_State;
	MODULE_BUS_PACKET * packet = Create_Module_Bus_Packet(MODEM_STATUS_INFO_EX, 0x01, 0x01, sizeof(rts), (unsigned char *)&rts);
	Module_Bus_Immediate_Send(packet, 1);
}

void Nimbelink_QBG96_Socket_Received_Data(Nimbelink_QBG96 * Device, uint8_t Socket, uint8_t * Data, uint16_t Data_Size)
{
//	printf("Received Data:\r\n Socket: %d\r\n Length: %d\r\n", (int)Socket, (int)Data_Size);

//	for(int i = 0; i < Data_Size; i++)
//		printf("0x%2X ", Data[i]);
//	printf("\r\n\r\n");


	if(_4G.Current_State == ePMG_4G_Receiving_Production_Test_Ping_Response)
	{
		// We are in production test so we don't care what data came back we are looking for ANY data to come back
		_4G.Current_State = ePMG_4G_Received_Production_Test_Ping_Response;
		return;
	}
	if(Socket == 0)
	{
		if(Data_Size <= I2C_MAX_PAYLOAD_SIZE)
		{
			MODULE_BUS_PACKET * packet = Create_Module_Bus_Packet(MODEM_ACI_SERVER_COMMAND, 0x01, 0x01, Data_Size, Data);
			Module_Bus_Buffered_Send(packet, 1);
		}
		else
		{
			// The data will not fit in one packet

			uint8_t Number_Of_Packets = Data_Size / I2C_MAX_PAYLOAD_SIZE;

			if(Data_Size - Number_Of_Packets * I2C_MAX_PAYLOAD_SIZE)
			{
				++Number_Of_Packets;
			}
			unsigned char * data_ptr = Data;

			for(uint8_t MsgNum = 1; Data_Size; MsgNum++)
			{
				uint8_t Packet_Size = MIN(Data_Size, I2C_MAX_PAYLOAD_SIZE);
				MODULE_BUS_PACKET * packet = Create_Module_Bus_Packet(MODEM_ACI_SERVER_COMMAND, MsgNum, Number_Of_Packets, Packet_Size, data_ptr);
				Module_Bus_Buffered_Send(packet, 1);

				data_ptr += Packet_Size;
				Data_Size -= Packet_Size;
				HAL_Delay(10);
			}
		}
	}
	else if(Socket == 1)
	{
		union
		{
			uint32_t Raw;
			struct
			{
				uint8_t Octet_1;
				uint8_t Octet_2;
				uint8_t Octet_3;
				uint8_t Octet_4;
			}Parts;
		}Address;



		Address.Parts.Octet_1 = ACI_IP_ADD_OCTET_4;
		Address.Parts.Octet_2 = ACI_IP_ADD_OCTET_3;
		Address.Parts.Octet_3 = ACI_IP_ADD_OCTET_2;
		Address.Parts.Octet_4 = ACI_IP_ADD_OCTET_1;

		uint32_t Src = Address.Raw;

		Address.Parts.Octet_1 = Radio->Radio_IP[0];
		Address.Parts.Octet_2 = Radio->Radio_IP[1];
		Address.Parts.Octet_3 = Radio->Radio_IP[2];
		Address.Parts.Octet_4 = Radio->Radio_IP[3];
		uint32_t Dst = Address.Raw;


		Send_TFTP_Packet_To_Controller(Src, 69, Dst, 0x4500, IPTProt_UDP, Data, Data_Size);
//		Send_TFTP_Packet_To_Controller(local_ip, 69, local_ip, 15628, IPTProt_UDP, Data, Data_Size);
	}
}

void Send_TFTP_Packet_To_Controller(uint32_t Src_Addr, uint16_t Src_Port, uint32_t Dst_Addr, uint16_t Dst_Port, uint8_t Protocol, uint8_t *Data, uint16_t Data_Size)
{
	// Module Bus UDP Packet Stack
	/************************************************************/
	/*      Module Bus Packet Header  --   4 bytes              */
	/*              IP Packet Header  --  20 bytes              */
	/*             UDP Packet Header  --   8 bytes              */
	/*                -- Payload --    0-255 bytes              */
	/*                    -- CRC --        1 byte               */
	/************************************************************/
	uint16_t IP_Pkt_Length = Data_Size + IP_HEADER_SIZE + UDP_HEADER_SIZE;


	uint16_t Ident = pmdGetMsTicks() & 0xFFFF;

	uint8_t Num_Mod_Bus_Packets = Get_Num_Module_Bus_Packets(IP_Pkt_Length);

	uint8_t mbus_payload_size = MIN(IP_Pkt_Length, I2C_MAX_PAYLOAD_SIZE);

	MODULE_BUS_PACKET *mbus_pkt = Create_Module_Bus_Packet(MODEM_UDP_PACKET, 1, Num_Mod_Bus_Packets, mbus_payload_size, NULL);

	IP_Hdr_t	*ip_header = (IP_Hdr_t*)mbus_pkt->Formatted_Data.Data;
	UDP_Hdr_t *udp_header = (UDP_Hdr_t*)(mbus_pkt->Formatted_Data.Data + IP_HEADER_SIZE);
	uint8_t *udp_pkt_payload = (uint8_t*)mbus_pkt->Formatted_Data.Data + IP_HEADER_SIZE + UDP_HEADER_SIZE;
	uint16_t udp_pkt_payload_length	= mbus_payload_size - IP_HEADER_SIZE - UDP_HEADER_SIZE;

	Build_IP_Packet_Header(	ip_header,
							Data_Size,
							Src_Addr,
							Dst_Addr,
							Protocol,
							Ident);

	if(Debug_Level >= DbgLvl_Extreme)
	{
		printf(">>MBus TX>> IP Packet Header:");
		Log_Hex_Data((uint8_t*)ip_header, IP_HEADER_SIZE);
	}

	Build_UDP_Packet_Header(ip_header,
							udp_header,
							Data,
							Data_Size,
							Src_Port,
							Dst_Port);


	// Send First Packet
	memcpy(udp_pkt_payload, Data, udp_pkt_payload_length);
	// Calculate the CRC
	Apply_Module_Bus_Packet_CRC(mbus_pkt);

	uint8_t modbus_overall_packet_length = mbus_payload_size + PACKET_HEADER_SIZE + CRC_SIZE;
	dlog(DbgLvl_Verbose, "ModBus Overall Packet Length:  %d\r\n", modbus_overall_packet_length);
	dlog(DbgLvl_Info, ">>MBus TX>> UDP pkt 1/%d (%d bytes)\r\n", Num_Mod_Bus_Packets, mbus_payload_size);

	if(Debug_Level >= DbgLvl_Verbose)
	{
		Log_Hex_Data(mbus_pkt->Raw_Data, modbus_overall_packet_length);
	}

//	printf("Received UDP packet from 4G:");
//	Log_Limited_Hex_Data((char*)mbus_pkt, udp_pkt_payload_length, 64);

	Module_Bus_Buffered_Send(mbus_pkt, 1);

	if(Data_Size < udp_pkt_payload_length)
	{
		dlog(DbgLvl_Error, "ERROR:  UDP packet underflow\r\n");
		return;
	}

	// TODO:  Need to get the ack back from the controller before we'll get the next packet in the multi-packet AC message.

	Data += udp_pkt_payload_length;
	Data_Size -= udp_pkt_payload_length;

	// Send Remaining Packets
	for(uint8_t MsgNum = 2; MsgNum <= Num_Mod_Bus_Packets; ++MsgNum)
	{
		mbus_payload_size	= MIN(Data_Size, I2C_MAX_PAYLOAD_SIZE);
		mbus_pkt = Create_Module_Bus_Packet(MODEM_UDP_PACKET, MsgNum, Num_Mod_Bus_Packets, mbus_payload_size, Data);

		// Calculate the CRC
		Apply_Module_Bus_Packet_CRC(mbus_pkt);


		dlog(DbgLvl_Info, ">>MBus TX>> UDP pkt %d/%d (%d bytes)\r\n", MsgNum, Num_Mod_Bus_Packets, mbus_payload_size);
		if(Debug_Level >= DbgLvl_Verbose)
		{
			Log_Hex_Data(mbus_pkt->Raw_Data, mbus_payload_size + PACKET_HEADER_SIZE + CRC_SIZE);
		}

		Module_Bus_Buffered_Send(mbus_pkt, 1);

		Data += mbus_payload_size;
		Data_Size -= mbus_payload_size;
	}
}
