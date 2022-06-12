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

#include "mcupmod.h"
/*
#include <microkvs/driver/STM32StorageBank.h>

//UART console
UART* g_cliUART = NULL;
Logger g_log;
UARTOutputStream g_uartStream;
DemoCLISessionContext g_uartCliContext;
Timer* g_logTimer;

//SPI interface
GPIOPin* g_spiCS = NULL;
SPI* g_spi = NULL;

//Ethernet interface
STM32EthernetInterface* g_eth;
bool g_hasRmiiErrata = false;
MACAddress g_macAddress;
IPv4Config g_ipconfig;
EthernetProtocol* g_ethStack;

//KVS
KVS* g_kvs;

void InitClocks();
void InitUART();
void InitCLI();
void InitLog();
void DetectHardware();
void InitSPI();
void InitKVS();
void InitEthernet();
bool TestEthernet(uint32_t num_frames);
void InitSSH();

uint8_t GetFPGAStatus();
*/
int main()
{
	//Copy .data from flash to SRAM (for some reason the default newlib startup won't do this??)
	memcpy(&__data_start, &__data_romstart, &__data_end - &__data_start + 1);

	GPIOPin led0_n(&GPIOB, 12, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin led1_n(&GPIOB, 13, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin led2_n(&GPIOG, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin led3_n(&GPIOG, 2, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	//LEDs are inverted, so bring them low to turn on
	led0_n = 0;
	led1_n = 0;
	led2_n = 0;
	led3_n = 0;

	/*
	//Hardware setup
	InitClocks();
	InitUART();
	InitCLI();
	InitLog();
	DetectHardware();
	InitSPI();
	InitKVS();
	InitEthernet();
	InitSSH();

	//Enable interrupts only after all setup work is done
	EnableInterrupts();

	//Show the initial prompt
	g_uartCliContext.PrintPrompt();

	//Main event loop
	int nextRxFrame = 0;
	uint32_t numRxFrames = 0;
	uint32_t numRxBad = 0;
	uint32_t nextAgingTick = 0;
	*/
	while(1)
	{
		/*
		//Wait for an interrupt
		//asm("wfi");

		//Poll for Ethernet frames
		auto frame = g_eth->GetRxFrame();
		if(frame != NULL)
			g_ethStack->OnRxFrame(frame);

		//Poll for UART input
		if(g_cliUART->HasInput())
			g_uartCliContext.OnKeystroke(g_cliUART->BlockingRead());

		//Check for aging on stuff once a second
		if(g_logTimer->GetCount() > nextAgingTick)
		{
			g_ethStack->OnAgingTick();
			nextAgingTick = g_logTimer->GetCount() + 10000;
		}
		*/
	}

	return 0;
}

/*
void InitClocks()
{
	//Configure the flash with wait states and prefetching before making any changes to the clock setup.
	//A bit of extra latency is fine, the CPU being faster than flash is not.
	Flash::SetConfiguration(true, true, 175, Flash::RANGE_2V7);

	//Set up the main system PLL. Input from HSI clock (16 MHz RC)
	RCCHelper::InitializePLLFromInternalOscillator(
		8,		//Pre-divider of 8 = 2 MHz input to PLL
		175,	//Multiply by 175 = 350 MHz VCO
		2,		//Divide VCO by 2 for CPU clock (175 MHz)
		10,		//Divide VCO by 10 for RNG (35 MHz)
		5,		//Divide VCO by 4 for MIPI DSI clock (70 MHz, but ignored since we don't have MIPI hardware)
		1,		//Divide CPU clock by 1 to get AHB clock (175 MHz)
		4,		//Divide AHB clock by 4 to get APB1 clock (43.75 MHz)
		2);		//Divide AHB clock by 2 to get APB2 clock (87.5 MHz)

	//TODO: Debug clock output on MCO1 (PA8) pmod, see 5.2.10?
}

void InitLog()
{
	//APB1 is 43.75 MHz but TIMxCLK = 2x that so 87.5 MHz.
	//Divide down to get 10 kHz ticks
	static Timer logtim(&TIM2, Timer::FEATURE_GENERAL_PURPOSE, 8750);
	g_logTimer = &logtim;

	g_log.Initialize(g_cliUART, &logtim);
	g_log("UART logging ready\n");
}

void DetectHardware()
{
	g_log("Identifying hardware\n");
	LogIndenter li(g_log);

	uint16_t rev = DBGMCU.IDCODE >> 16;
	uint16_t device = DBGMCU.IDCODE & 0xfff;

	if(device == 0x451)
	{
		//Look up the stepping number
		const char* srev = NULL;
		switch(rev)
		{
			case 0x1000:
				srev = "A";
				g_hasRmiiErrata = true;
				break;

			case 0x1001:
				srev = "Z";
				break;

			default:
				srev = "(unknown)";
		}

		uint8_t pkg = (PKG_ID >> 8) & 0x7;
		switch(pkg)
		{
			case 7:
				g_log("STM32F767 / 777 LQFP208/TFBGA216 rev %s (0x%04x)\n", srev, rev);
				break;
			case 6:
				g_log("STM32F769 / 779 LQFP208/TFBGA216 rev %s (0x%04x)\n", srev, rev);
				break;
			case 5:
				g_log("STM32F767 / 777 LQFP176 rev %s (0x%04x)\n", srev, rev);
				break;
			case 4:
				g_log("STM32F769 / 779 LQFP176 rev %s (0x%04x)\n", srev, rev);
				break;
			case 3:
				g_log("STM32F778 / 779 WLCSP180 rev %s (0x%04x)\n", srev, rev);
				break;
			case 2:
				g_log("STM32F767 / 777 LQFP144 rev %s (0x%04x)\n", srev, rev);
				break;
			case 1:
				g_log("STM32F767 / 777 LQFP100 rev %s (0x%04x)\n", srev, rev);
				break;
			default:
				g_log("Unknown/reserved STM32F76x/F77x rev %s (0x%04x)\n", srev, rev);
				break;
		}
		g_log("512 kB total SRAM, 128 kB DTCM, 16 kB ITCM, 4 kB backup SRAM\n");
		g_log("%d kB Flash\n", F_ID);

		//U_ID fields documented in 45.1 of STM32 programming manual
		uint16_t waferX = U_ID[0] >> 16;
		uint16_t waferY = U_ID[0] & 0xffff;
		uint8_t waferNum = U_ID[1] & 0xff;
		char waferLot[8] =
		{
			static_cast<char>((U_ID[1] >> 24) & 0xff),
			static_cast<char>((U_ID[1] >> 16) & 0xff),
			static_cast<char>((U_ID[1] >> 8) & 0xff),
			static_cast<char>((U_ID[2] >> 24) & 0xff),
			static_cast<char>((U_ID[2] >> 16) & 0xff),
			static_cast<char>((U_ID[2] >> 8) & 0xff),GPIOPin spi_sck(&GPIOH, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
			static_cast<char>((U_ID[2] >> 0) & 0xff),
			'\0'
		};
		g_log("Lot %s, wafer %d, die (%d, %d)\n", waferLot, waferNum, waferX, waferY);

		if(g_hasRmiiErrata)
			g_log(Logger::WARNING, "RMII RXD0 errata present\n");
	}
	else
		g_log(Logger::WARNING, "Unknown device (0x%06x)\n", device);
}

void InitUART()
{
	//Initialize the UART for local console: 115.2 Kbps using PA0 for UART4 transmit and PA3 for USART2 RX
	//TODO: nice interface for enabling UART interrupts
	GPIOPin uart_tx(&GPIOA, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 8);
	GPIOPin uart_rx(&GPIOA, 3, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 7);
	static UART uart(&UART4, &USART2, 380);
	volatile uint32_t* NVIC_ISER1 = (volatile uint32_t*)(0xe000e104);
	*NVIC_ISER1 = 0x40;
	g_cliUART = &uart;

	//Clear screen and move cursor to X0Y0
	uart.Printf("\x1b[2J\x1b[0;0H");
}

void InitCLI()
{
	g_log("Initializing CLI\n");

	//Initialize the CLI on the console UART interface
	g_uartStream.Initialize(g_cliUART);
	g_uartCliContext.Initialize(&g_uartStream, "admin");

	g_log("IP address not configured, defaulting to 192.168.1.42\n");
}

void InitSPI()
{
	g_log("Initializing SPI interface\n");

	static GPIOPin spi_cs_n(&GPIOH, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);
	GPIOPin spi_sck(&GPIOH, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	GPIOPin spi_miso(&GPIOH, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	GPIOPin spi_mosi(&GPIOF, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	g_spiCS = &spi_cs_n;
	spi_cs_n = 1;

	//APB2 is 87.5 MHz, /4 is 21.875 MHz
	static SPI spi(&SPI5, true, 4);
	g_spi = &spi;
}
*/
void InitKVS()
{
	/*g_log("Initializing microkvs key-value store\n");
	LogIndenter li(g_log);*/

	/*
		Use sectors 10 and 11 of main flash (in single bank mode) for a 256 kB microkvs

		Each log entry is 28 bytes, and we want to allocate ~25% of storage to the log since our objects are pretty
		small (SSH keys, IP addresses, etc). A 2048-entry log is a nice round number, and comes out to 56 kB or 21%,
		leaving the remaining 200 kB or 79% for data.
	 */
	/*static STM32StorageBank left(reinterpret_cast<uint8_t*>(0x08180000), 0x40000);
	static STM32StorageBank right(reinterpret_cast<uint8_t*>(0x081c0000), 0x40000);
	static KVS kvs(&left, &right, 2048);
	g_kvs = &kvs;

	g_log("Log area:  %d free entries\n", g_kvs->GetFreeLogEntries());
	uint32_t space = g_kvs->GetFreeDataSpace();
	g_log("Data area: %d.%02d kB free space\n", space/1024, (space % 1024) * 100 / 1024);

	//Load hostname, if we have it
	if(!g_kvs->ReadObject("hostname", (uint8_t*)g_hostname, sizeof(g_hostname)-1))
	{
		g_log("Hostname not configured, defaulting to \"demo\"\n");
		strncpy(g_hostname, "demo", sizeof(g_hostname));
	}*/
}

/*
uint8_t GetFPGAStatus()
{
	//Get the status register
	*g_spiCS = 0;
	g_spi->BlockingWrite(REG_STATUS);
	uint8_t sr = g_spi->BlockingRead();
	*g_spiCS = 1;

	return sr;
}

void InitEthernet()
{
	g_log("Initializing Ethernet\n");
	LogIndenter li(g_log);

	//Wait for the FPGA to be up and have our MAC address
	g_log("Polling FPGA status\n");
	while(true)
	{
		auto sr = GetFPGAStatus();
		if(sr & 1)
		{
			g_log(Logger::ERROR, "FPGA failed to get MAC address\n");
			while(1)
			{}
		}

		//address is ready
		if(sr & 2)
			break;
	}
	g_log("FPGA is up and has MAC address ready for us\n");

	//Read the MAC address from the FPGA
	*g_spiCS = 0;
	g_spi->BlockingWrite(REG_MAC_ADDR);
	for(int i=0; i<6; i++)
		g_macAddress[i] = g_spi->BlockingRead();
	*g_spiCS = 1;

	g_log("Our MAC address is %02x:%02x:%02x:%02x:%02x:%02x\n",
		g_macAddress[0], g_macAddress[1], g_macAddress[2], g_macAddress[3], g_macAddress[4], g_macAddress[5]);

	//Initialize the Ethernet pins. AF11 on all pins
	g_log("Initializing Ethernet pins\n");
	GPIOPin rmii_refclk(&GPIOA, 1, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_mdio(&GPIOA, 2, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_crs_dv(&GPIOA, 7, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_tx_en(&GPIOB, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 11);
	GPIOPin rmii_txd0(&GPIOB, 12, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 11);
	GPIOPin rmii_txd1(&GPIOB, 13, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 11);
	GPIOPin rmii_mdc(&GPIOC, 1, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_rxd0(&GPIOC, 4, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_rxd1(&GPIOC, 5, GPIOPin::MODE_PERIPHERAL, 11);

	//Enable SYSCFG before changing any settings on it
	RCC.APB2ENR |= RCC_APB2_SYSCFG;

	//Ignore the MDIO bus for now

	//Read IP address configuration
	//Default to 192.168.1.42 if not configured
	//This is so much nicer looking with C++ 20 but Debian's arm-none-eabi-gcc cross compiler is currently stuck at
	//C++ 17 even though the host compiler does 20 just fine...
	if(!g_kvs->ReadObject("ip.addr", g_ipconfig.m_address.m_octets, 4))
	{
		g_log("IP address not configured, defaulting to 192.168.1.42\n");
		g_ipconfig.m_address.m_octets[0] = 192;
		g_ipconfig.m_address.m_octets[1] = 168;
		g_ipconfig.m_address.m_octets[2] = 1;
		g_ipconfig.m_address.m_octets[3] = 42;
	}

	//Read subnet mask
	//Default to /24 if not configured
	if(!g_kvs->ReadObject("ip.netmask", g_ipconfig.m_netmask.m_octets, 4))
	{
		g_log("Subnet mask not configured, defaulting to 255.255.255.0\n");
		g_ipconfig.m_netmask.m_octets[0] = 255;
		g_ipconfig.m_netmask.m_octets[1] = 255;
		g_ipconfig.m_netmask.m_octets[2] = 255;
		g_ipconfig.m_netmask.m_octets[3] = 0;
	}

	//Read gateway address mask
	//Default to 192.168.1.1 if not configured
	if(!g_kvs->ReadObject("ip.gateway", g_ipconfig.m_gateway.m_octets, 4))
	{
		g_log("Default gateway not configured, defaulting to 192.168.1.1\n");
		g_ipconfig.m_gateway.m_octets[0] = 192;
		g_ipconfig.m_gateway.m_octets[1] = 168;
		g_ipconfig.m_gateway.m_octets[2] = 1;
		g_ipconfig.m_gateway.m_octets[3] = 1;
	}

	//Calculate broadcast address
	for(int i=0; i<4; i++)
		g_ipconfig.m_broadcast.m_octets[i] = g_ipconfig.m_address.m_octets[i] | ~g_ipconfig.m_netmask.m_octets[i];

	//Set up all of the SFRs for the Ethernet IP itself
	g_log("Initializing MAC and DMA\n");
	static STM32EthernetInterface enet;
	g_eth = &enet;

	//Quick sanity check to make sure the link is up
	TestEthernet(25);

	//ARP cache
	static ARPCache arpCache;

	//Protocol stacks
	static EthernetProtocol eth(*g_eth, g_macAddress);
	static ARPProtocol arp(eth, g_ipconfig.m_address, arpCache);
	static IPv4Protocol ipv4(eth, g_ipconfig, arpCache);
	static ICMPv4Protocol icmpv4(ipv4);
	static DemoTCPProtocol tcp(&ipv4);

	//Register protocol handlers
	eth.UseARP(&arp);
	eth.UseIPv4(&ipv4);
	ipv4.UseICMPv4(&icmpv4);
	ipv4.UseTCP(&tcp);

	//Save the stack so we can use it later
	g_ethStack = &eth;
}

void InitSSH()
{
	g_log("Initializing SSH server\n");
	LogIndenter li(g_log);

	unsigned char pub[ECDSA_KEY_SIZE] = {0};
	unsigned char priv[ECDSA_KEY_SIZE] = {0};

	bool found = true;
	if(!g_kvs->ReadObject("ssh.hostpub", pub, ECDSA_KEY_SIZE))
		found = false;
	if(!g_kvs->ReadObject("ssh.hostpriv", priv, ECDSA_KEY_SIZE))
		found = false;

	if(found)
	{
		g_log("Using existing SSH host key\n");
		CryptoEngine::SetHostKey(pub, priv);
	}

	else
	{
		g_log("No SSH host key in flash, generating new key pair\n");
		STM32CryptoEngine tmp;
		tmp.GenerateHostKey();

		if(!g_kvs->StoreObject("ssh.hostpub", CryptoEngine::GetHostPublicKey(), ECDSA_KEY_SIZE))
			g_log(Logger::ERROR, "Unable to store SSH host public key to flash\n");
		if(!g_kvs->StoreObject("ssh.hostpriv", CryptoEngine::GetHostPrivateKey(), ECDSA_KEY_SIZE))
			g_log(Logger::ERROR, "Unable to store SSH host private key to flash\n");
	}

	char buf[64] = {0};
	STM32CryptoEngine tmp;
	tmp.GetHostKeyFingerprint(buf, sizeof(buf));
	g_log("ED25519 key fingerprint is SHA256:%s.\n", buf);
}

bool TestEthernet(uint32_t num_frames)
{
	g_log("Testing %d frames\n", num_frames);
	LogIndenter li(g_log);

	g_log("Putting FPGA in test mode\n");
	*g_spiCS = 0;
	g_spi->BlockingWrite(REG_RX_DISABLE);
	g_spi->WaitForWrites();
	*g_spiCS = 1;

	const int timeout = 50;

	for(uint32_t i=0; i<num_frames; i++)
	{
		//Ask for the frame
		*g_spiCS = 0;
		g_spi->BlockingWrite(REG_SEND_TEST);
		g_spi->WaitForWrites();
		*g_spiCS = 1;

		//Get current time
		auto tim = g_logTimer->GetCount();

		while(true)
		{
			//Check for a frame
			auto frame = g_eth->GetRxFrame();
			if(frame != NULL)
			{
				if(frame->Length() != 64)
				{
					g_log(Logger::ERROR, "Bad length on frame %d (%d bytes, expected 64)\n", i, frame->Length());
					return false;
				}

				g_eth->ReleaseRxFrame(frame);
				break;
			}

			//If we see a CRC error in the counters but the frame didn't get DMA'd, it's still a fail
			if(EMAC.MMCRFCECR != 0)
			{
				g_log(Logger::ERROR, "Bad CRC on frame %d (reported by counters)\n", i);
				break;
				return false;
			}

			//Time out if it's been too long
			auto delta = g_logTimer->GetCount() - tim;
			if(delta >= timeout)
			{
				g_log(Logger::ERROR, "Timed out after %d ms waiting for frame %d\n", timeout / 10, i);
				return false;
			}
		}
	}

	g_log("Test successful, enabling RX path in FPGA\n");

	*g_spiCS = 0;
	g_spi->BlockingWrite(REG_RX_ENABLE);
	g_spi->WaitForWrites();
	*g_spiCS = 1;
	return true;
}
*/
