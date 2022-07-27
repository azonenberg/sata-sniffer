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
#include "SnifferCLISessionContext.h"
#include <ctype.h>

char g_hostname[33] = {0};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Command table

//List of all valid commands
enum cmdid_t
{
	CMD_ADDRESS,
	CMD_ALL,
	//CMD_ARP,
	//CMD_CACHE,
	CMD_DEFAULT_GATEWAY,
	CMD_EXIT,
	//CMD_FINGERPRINT,
	CMD_FLASH,
	//CMD_HARDWARE,
	CMD_HOSTNAME,
	CMD_IP,
	CMD_RELOAD,
	CMD_ROUTE,
	CMD_SHOW,
	//CMD_SSH,
	CMD_ZEROIZE,
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "hostname"

static const clikeyword_t g_hostnameCommands[] =
{
	{"<string>",		FREEFORM_TOKEN,			NULL,						"New host name"},
	{NULL,				INVALID_COMMAND,		NULL,						NULL}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "ip"

static const clikeyword_t g_ipAddressCommands[] =
{
	{"<string>",		FREEFORM_TOKEN,			NULL,						"New IPv4 address and subnet mask in x.x.x.x/yy format"},
	{NULL,				INVALID_COMMAND,		NULL,						NULL}
};

static const clikeyword_t g_defaultGatewayCommands[] =
{
	{"<string>",		FREEFORM_TOKEN,			NULL,						"New IPv4 default gateway"},
	{NULL,				INVALID_COMMAND,		NULL,						NULL}
};

static const clikeyword_t g_ipCommands[] =
{
	{"address",			CMD_ADDRESS,			g_ipAddressCommands,		"Set the IPv4 address and subnet mask"},
	{"default-gateway",	CMD_DEFAULT_GATEWAY,	g_defaultGatewayCommands,	"Set the IPv4 default route"},

	{NULL,				INVALID_COMMAND,		NULL,						NULL}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "show"
/*
static const clikeyword_t g_showArpCommands[] =
{
	{"cache",			CMD_CACHE,				NULL,						"Show contents of the ARP cache"},

	{NULL,				INVALID_COMMAND,		NULL,						NULL}
};
*/
static const clikeyword_t g_showIpCommands[] =
{
	{"address",			CMD_ADDRESS,			NULL,						"Show the IPv4 address and subnet mask"},
	{"route",			CMD_ROUTE,				NULL,						"Show the IPv4 routing table"},

	{NULL,				INVALID_COMMAND,		NULL,						NULL}
};
/*
static const clikeyword_t g_showSshCommands[] =
{
	{"fingerprint",		CMD_FINGERPRINT,		NULL,						"Show the SSH host key fingerprint (in OpenSSH base64 SHA256 format)"},

	{NULL,				INVALID_COMMAND,		NULL,						NULL}
};
*/
static const clikeyword_t g_showCommands[] =
{
	//{"arp",				CMD_ARP,				g_showArpCommands,			"Print ARP information"},
	{"flash",			CMD_FLASH,				NULL,						"Print contents and size of config storage"},
	//{"hardware",		CMD_HARDWARE,			NULL,						"Print hardware information"},
	{"ip",				CMD_IP,					g_showIpCommands,			"Print IPv4 information"},
	//{"ssh",				CMD_SSH,				g_showSshCommands,			"Print SSH information"},

	{NULL,				INVALID_COMMAND,		NULL,	NULL}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "zeroize"

static const clikeyword_t g_zeroizeCommands[] =
{
	{"all",				CMD_ALL,				NULL,						"Confirm erasing all data"},
	{NULL,				INVALID_COMMAND,		NULL,						NULL}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Top level command list

static const clikeyword_t g_rootCommands[] =
{
	{"exit",			CMD_EXIT,				NULL,						"Log out"},
	{"hostname",		CMD_HOSTNAME,			g_hostnameCommands,			"Change the host name"},
	{"ip",				CMD_IP,					g_ipCommands,				"Configure IP addresses"},
	{"reload",			CMD_RELOAD,				NULL,						"Restart the system"},
	{"show",			CMD_SHOW,				g_showCommands,				"Print information"},
	{"zeroize",			CMD_ZEROIZE,			g_zeroizeCommands,			"Erase all configuration data and reset"},

	{NULL,				INVALID_COMMAND,		NULL,						NULL}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

SnifferCLISessionContext::SnifferCLISessionContext()
	: CLISessionContext(g_rootCommands)
	, m_stream(NULL)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prompt

void SnifferCLISessionContext::PrintPrompt()
{
	m_stream->Printf("%s@%s$ ", m_username, g_hostname);
	m_stream->Flush();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Top level command dispatch

void SnifferCLISessionContext::OnExecute()
{
	switch(m_command[0].m_commandID)
	{
		case CMD_EXIT:
			m_stream->Flush();
			m_stream->Disconnect();
			break;

		case CMD_HOSTNAME:
			SetHostName(m_command[1].m_text);
			break;

		case CMD_IP:
			switch(m_command[1].m_commandID)
			{
				case CMD_ADDRESS:
					OnIPAddress(m_command[2].m_text);
					break;

				case CMD_DEFAULT_GATEWAY:
					OnDefaultGateway(m_command[2].m_text);
					break;

				default:
					break;
			}
			break;

		case CMD_RELOAD:
			OnReload();
			break;

		case CMD_SHOW:
			OnShowCommand();
			break;

		case CMD_ZEROIZE:
			if(m_command[1].m_commandID == CMD_ALL)
				OnZeroize();
			break;

		default:
			break;
	}

	m_stream->Flush();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "hostname"

void SnifferCLISessionContext::SetHostName(const char* name)
{
	strncpy(g_hostname, name, sizeof(g_hostname)-1);
	g_kvs->StoreObject("hostname", (uint8_t*)g_hostname, sizeof(g_hostname)-1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "ip"

void SnifferCLISessionContext::OnDefaultGateway(const char* ipstring)
{
	int len = strlen(ipstring);

	int nfield = 0;
	unsigned int fields[4] = {0};

	//Parse
	bool fail = false;
	for(int i=0; i<len; i++)
	{
		//Dot = move to next field
		if( (ipstring[i] == '.') && (nfield < 3) )
			nfield ++;

		//Digit = update current field
		else if(isdigit(ipstring[i]))
			fields[nfield] = (fields[nfield] * 10) + (ipstring[i] - '0');

		else
		{
			fail = true;
			break;
		}
	}

	//Validate
	if(nfield != 3)
		fail = true;
	for(int i=0; i<4; i++)
	{
		if(fields[i] > 255)
		{
			fail = true;
			break;
		}
	}
	if(fail)
	{
		m_stream->Printf("Usage: ip default-gateway x.x.x.x\n");
		return;
	}

	//Set the IP
	for(int i=0; i<4; i++)
		g_ipConfig.m_gateway.m_octets[i] = fields[i];

	//Write the new configuration to flash
	if(!g_kvs->StoreObject("ip.gateway", g_ipConfig.m_gateway.m_octets, 4))
		g_log(Logger::ERROR, "Failed to write gateway to flash\n");

	//Push it to the FPGA
	g_qspi->BlockingWrite(REG_GATEWAY, 0, g_ipConfig.m_gateway.m_octets, sizeof(IPv4Address));
}

void SnifferCLISessionContext::OnIPAddress(const char* ipstring)
{
	int len = strlen(ipstring);

	int nfield = 0;	//0-3 = IP, 4 = netmask
	unsigned int fields[5] = {0};

	//Parse
	bool fail = false;
	for(int i=0; i<len; i++)
	{
		//Dot = move to next field
		if( (ipstring[i] == '.') && (nfield < 3) )
			nfield ++;

		//Slash = move to netmask
		else if( (ipstring[i] == '/') && (nfield == 3) )
			nfield ++;

		//Digit = update current field
		else if(isdigit(ipstring[i]))
			fields[nfield] = (fields[nfield] * 10) + (ipstring[i] - '0');

		else
		{
			fail = true;
			break;
		}
	}

	//Validate
	if(nfield != 4)
		fail = true;
	for(int i=0; i<4; i++)
	{
		if(fields[i] > 255)
		{
			fail = true;
			break;
		}
	}
	if( (fields[4] > 32) || (fields[4] == 0) )
		fail = true;
	if(fail)
	{
		m_stream->Printf("Usage: ip address x.x.x.x/yy\n");
		return;
	}

	//Set the IP
	for(int i=0; i<4; i++)
		g_ipConfig.m_address.m_octets[i] = fields[i];

	//Calculate the netmask
	uint32_t mask = 0xffffffff << (32 - fields[4]);
	g_ipConfig.m_netmask.m_octets[0] = (mask >> 24) & 0xff;
	g_ipConfig.m_netmask.m_octets[1] = (mask >> 16) & 0xff;
	g_ipConfig.m_netmask.m_octets[2] = (mask >> 8) & 0xff;
	g_ipConfig.m_netmask.m_octets[3] = (mask >> 0) & 0xff;

	//Calculate the broadcast address
	for(int i=0; i<4; i++)
		g_ipConfig.m_broadcast.m_octets[i] = g_ipConfig.m_address.m_octets[i] | ~g_ipConfig.m_netmask.m_octets[i];

	//Write the new IP configuration to flash
	if(!g_kvs->StoreObject("ip.addr", g_ipConfig.m_address.m_octets, 4))
		g_log(Logger::ERROR, "Failed to write IP address to flash\n");
	if(!g_kvs->StoreObject("ip.netmask", g_ipConfig.m_netmask.m_octets, 4))
		g_log(Logger::ERROR, "Failed to write IP address to flash\n");

	//Push it to the FPGA
	g_qspi->BlockingWrite(REG_IP_ADDRESS, 0, g_ipConfig.m_address.m_octets, sizeof(IPv4Address));
	g_qspi->BlockingWrite(REG_SUBNET_MASK, 0, g_ipConfig.m_netmask.m_octets, sizeof(IPv4Address));
	g_qspi->BlockingWrite(REG_BROADCAST, 0, g_ipConfig.m_broadcast.m_octets, sizeof(IPv4Address));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "reload"

void SnifferCLISessionContext::OnReload()
{
	g_log("Reload requested\n");
	SCB.AIRCR = 0x05fa0004;
	while(1)
	{}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "show"

void SnifferCLISessionContext::OnShowCommand()
{
	switch(m_command[1].m_commandID)
	{
		/*case CMD_ARP:
			switch(m_command[2].m_commandID)
			{
				case CMD_CACHE:
					ShowARPCache();
					break;

				default:
					break;
			}
			break;*/

		case CMD_FLASH:
			ShowFlash();
			break;

		/*case CMD_HARDWARE:
			ShowHardware();
			break;*/

		case CMD_IP:
			switch(m_command[2].m_commandID)
			{
				case CMD_ADDRESS:
					ShowIPAddr();
					break;

				case CMD_ROUTE:
					ShowIPRoute();
					break;

				default:
					break;
			}
			break;

		/*case CMD_SSH:
			switch(m_command[2].m_commandID)
			{
				case CMD_FINGERPRINT:
					ShowSSHFingerprint();
					break;

				default:
					break;
			}
			break;*/
	}
}
/*
void SnifferCLISessionContext::ShowARPCache()
{
	auto cache = g_ethStack->GetARP()->GetCache();

	uint32_t ways = cache->GetWays();
	uint32_t lines = cache->GetLines();
	m_stream->Printf("ARP cache is %d ways of %d lines, %d spaces total\n", ways, lines, ways*lines);

	m_stream->Printf("Expiration  HWaddress           Address\n");

	for(uint32_t i=0; i<ways; i++)
	{
		auto way = cache->GetWay(i);
		for(uint32_t j=0; j<lines; j++)
		{
			auto& line = way->m_lines[j];
			if(line.m_valid)
			{
				m_stream->Printf("%10d  %02x:%02x:%02x:%02x:%02x:%02x   %d.%d.%d.%d\n",
					line.m_lifetime,
					line.m_mac[0], line.m_mac[1], line.m_mac[2], line.m_mac[3], line.m_mac[4], line.m_mac[5],
					line.m_ip.m_octets[0], line.m_ip.m_octets[1], line.m_ip.m_octets[2], line.m_ip.m_octets[3]
				);
			}
		}
	}
}
*/
void SnifferCLISessionContext::ShowFlash()
{
	//Print info about the flash memory in general
	m_stream->Printf("Flash configuration storage is 2 banks of %d kB\n", g_kvs->GetBlockSize() / 1024);
	if(g_kvs->IsLeftBankActive())
		m_stream->Printf("    Active bank: Left\n");
	else
		m_stream->Printf("    Active bank: Right\n");
	m_stream->Printf("    Log area:    %6d / %6d entries free (%d %%)\n",
		g_kvs->GetFreeLogEntries(),
		g_kvs->GetLogCapacity(),
		g_kvs->GetFreeLogEntries()*100 / g_kvs->GetLogCapacity());
	m_stream->Printf("    Data area:   %6d / %6d kB free      (%d %%)\n",
		g_kvs->GetFreeDataSpace() / 1024,
		g_kvs->GetDataCapacity() / 1024,
		g_kvs->GetFreeDataSpace() * 100 / g_kvs->GetDataCapacity());

	//Dump directory listing
	const uint32_t nmax = 32;
	KVSListEntry list[nmax];
	uint32_t nfound = g_kvs->EnumObjects(list, nmax);
	m_stream->Printf("    Objects:\n");
	m_stream->Printf("        Key               Size  Revisions\n");
	int size = 0;
	for(uint32_t i=0; i<nfound; i++)
	{
		m_stream->Printf("        %-16s %5d  %d\n", list[i].key, list[i].size, list[i].revs);
		size += list[i].size;
	}
	m_stream->Printf("    %d objects total (%d.%02d kB)\n",
		nfound,
		size/1024, (size % 1024) * 100 / 1024);
}
/*
void SnifferCLISessionContext::ShowHardware()
{
	uint16_t rev = DBGMCU.IDCODE >> 16;
	uint16_t device = DBGMCU.IDCODE & 0xfff;

	m_stream->Printf("MCU:\n");
	if(device == 0x451)
	{
		//Look up the stepping number
		const char* srev = NULL;
		switch(rev)
		{
			case 0x1000:
				srev = "A";
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
				m_stream->Printf("    STM32F767 / 777 LQFP208/TFBGA216 rev %s (0x%04x)\n", srev, rev);
				break;
			case 6:
				m_stream->Printf("    STM32F769 / 779 LQFP208/TFBGA216 rev %s (0x%04x)\n", srev, rev);
				break;
			case 5:
				m_stream->Printf("    STM32F767 / 777 LQFP176 rev %s (0x%04x)\n", srev, rev);
				break;
			case 4:
				m_stream->Printf("    STM32F769 / 779 LQFP176 rev %s (0x%04x)\n", srev, rev);
				break;
			case 3:
				m_stream->Printf("    STM32F778 / 779 WLCSP180 rev %s (0x%04x)\n", srev, rev);
				break;
			case 2:
				m_stream->Printf("    STM32F767 / 777 LQFP144 rev %s (0x%04x)\n", srev, rev);
				break;
			case 1:
				m_stream->Printf("    STM32F767 / 777 LQFP100 rev %s (0x%04x)\n", srev, rev);
				break;
			default:
				m_stream->Printf("    Unknown/reserved STM32F76x/F77x rev %s (0x%04x)\n", srev, rev);
				break;
		}
		m_stream->Printf("    512 kB total SRAM, 128 kB DTCM, 16 kB ITCM, 4 kB backup SRAM\n");
		m_stream->Printf("    %d kB Flash\n", F_ID);

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
			static_cast<char>((U_ID[2] >> 8) & 0xff),
			static_cast<char>((U_ID[2] >> 0) & 0xff),
			'\0'
		};
		m_stream->Printf("    Lot %s, wafer %d, die (%d, %d)\n", waferLot, waferNum, waferX, waferY);

		if(g_hasRmiiErrata)
			m_stream->Printf("    RMII RXD0 errata present\n");
	}
	else
		m_stream->Printf("Unknown device (0x%06x)\n", device);

	//Print CPU info
	if( (SCB.CPUID & 0xff00fff0) == 0x4100c270 )
	{
		m_stream->Printf("ARM Cortex-M7 r%dp%d\n", (SCB.CPUID >> 20) & 0xf, (SCB.CPUID & 0xf));
		if(CPUID.CLIDR & 2)
		{
			m_stream->Printf("    L1 data cache present\n");
			CPUID.CCSELR = 0;

			int sets = ((CPUID.CCSIDR >> 13) & 0x7fff) + 1;
			int ways = ((CPUID.CCSIDR >> 3) & 0x3ff) + 1;
			int words = 1 << ((CPUID.CCSIDR & 3) + 2);
			int total = (sets * ways * words * 4) / 1024;
			m_stream->Printf("        %d sets, %d ways, %d words per line, %d kB total\n",
				sets, ways, words, total);
		}
		if(CPUID.CLIDR & 1)
		{
			m_stream->Printf("    L1 instruction cache present\n");
			CPUID.CCSELR = 1;

			int sets = ((CPUID.CCSIDR >> 13) & 0x7fff) + 1;
			int ways = ((CPUID.CCSIDR >> 3) & 0x3ff) + 1;
			int words = 1 << ((CPUID.CCSIDR & 3) + 2);
			int total = (sets * ways * words * 4) / 1024;
			m_stream->Printf("        %d sets, %d ways, %d words per line, %d kB total\n",
				sets, ways, words, total);
		}
	}
	else
		m_stream->Printf("Unknown CPU (0x%08x)\n", SCB.CPUID);

	m_stream->Printf("Ethernet MAC address is %02x:%02x:%02x:%02x:%02x:%02x\n",
		g_macAddress[0], g_macAddress[1], g_macAddress[2], g_macAddress[3], g_macAddress[4], g_macAddress[5]);
}
*/
void SnifferCLISessionContext::ShowIPAddr()
{
	m_stream->Printf("IPv4 address: %d.%d.%d.%d\n",
		g_ipConfig.m_address.m_octets[0],
		g_ipConfig.m_address.m_octets[1],
		g_ipConfig.m_address.m_octets[2],
		g_ipConfig.m_address.m_octets[3]
	);

	m_stream->Printf("Subnet mask:  %d.%d.%d.%d\n",
		g_ipConfig.m_netmask.m_octets[0],
		g_ipConfig.m_netmask.m_octets[1],
		g_ipConfig.m_netmask.m_octets[2],
		g_ipConfig.m_netmask.m_octets[3]
	);

	m_stream->Printf("Broadcast:    %d.%d.%d.%d\n",
		g_ipConfig.m_broadcast.m_octets[0],
		g_ipConfig.m_broadcast.m_octets[1],
		g_ipConfig.m_broadcast.m_octets[2],
		g_ipConfig.m_broadcast.m_octets[3]
	);
}

void SnifferCLISessionContext::ShowIPRoute()
{
	m_stream->Printf("IPv4 routing table\n");
	m_stream->Printf("Destination     Gateway\n");
	m_stream->Printf("0.0.0.0         %d.%d.%d.%d\n",
		g_ipConfig.m_gateway.m_octets[0],
		g_ipConfig.m_gateway.m_octets[1],
		g_ipConfig.m_gateway.m_octets[2],
		g_ipConfig.m_gateway.m_octets[3]);
}
/*
void SnifferCLISessionContext::ShowSSHFingerprint()
{
	char buf[64] = {0};
	STM32CryptoEngine tmp;
	tmp.GetHostKeyFingerprint(buf, sizeof(buf));
	m_stream->Printf("ED25519 key fingerprint is SHA256:%s.\n", buf);
}
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "zeroize"

void SnifferCLISessionContext::OnZeroize()
{
	g_kvs->WipeAll();
	OnReload();
}
