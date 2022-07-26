/***********************************************************************************************************************
*                                                                                                                      *
* sata-sniffer v0.1                                                                                                    *
*                                                                                                                      *
* Copyright (c) 2022 Andrew D. Zonenberg and contributors                                                              *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#ifndef sshcli_h
#define sshcli_h

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stm32.h>

#include <peripheral/Flash.h>
#include <peripheral/GPIO.h>
#include <peripheral/I2C.h>
#include <peripheral/OctoSPI.h>
#include <peripheral/OctoSPIManager.h>
#include <peripheral/Power.h>
/*
#include <peripheral/RCC.h>
#include <peripheral/SPI.h>
*/
#include <peripheral/Timer.h>
#include <peripheral/UART.h>
#include <util/Logger.h>
#include <util/FIFO.h>
#include <cli/UARTOutputStream.h>
/*
#include <staticnet-config.h>
#include <staticnet/stack/staticnet.h>
#include <staticnet/drivers/stm32/STM32EthernetInterface.h>
#include <staticnet/drivers/stm32/STM32CryptoEngine.h>
*/
#include <microkvs/kvs/KVS.h>
#include "SnifferCLISessionContext.h"
/*
#include "DemoTCPProtocol.h"
#include "DemoSSHTransportServer.h"
*/
extern UART* g_cliUART;
extern Logger g_log;
/*
extern UARTOutputStream g_uartStream;
extern DemoCLISessionContext g_uartCliContext;
extern GPIOPin* g_spiCS;
extern SPI* g_spi;
*/
extern KVS* g_kvs;

extern char g_hostname[33];
/*
extern MACAddress g_macAddress;
extern IPv4Config g_ipconfig;
extern EthernetProtocol* g_ethStack;

//Register IDs for the FPGA
enum regids
{
	REG_STATUS			= 0x00,
	REG_MAC_ADDR		= 0x01,
	REG_EEPROM_SERIAL	= 0x02,
	REG_SEND_TEST		= 0x03,
	REG_RX_DISABLE		= 0x04,
	REG_RX_ENABLE		= 0x05
};
*/

#endif
