#include <stdio.h>
#include <cstring>
#include <string>
#include <iomanip>
#include <sstream>

// ModelSim header file
#include "mti.h"

// boost header files
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <vector>

using boost::asio::ip::udp;

typedef struct 
{
    mtiSignalIdT reset;
    mtiSignalIdT rxclk;
    MTISIGNALIDT TXCLK;
    MTIDRIVERIDT GMII_TXD;
    MTIDRIVERIDT GMII_TX_DV;
    mtiDriverIdT gmii_tx_er;
    mtiSignalIdT gmii_rxd;
    mtiSignalIdT gmii_rx_dv;
    mtiDriverIdT gmii_rx_er;
} inst_rec;

typedef enum boolean_tag 
{
    BOOL_FALSE,
    BOOL_TRUE
} boolean;



boost::array<unsigned char, 4> crc(std::vector<unsigned char>& eth_packet)
{
    const unsigned int crc_table[] =
    {
        0x4DBDF21C, 0x500AE278, 0x76D3D2D4, 0x6B64C2B0,
        0x3B61B38C, 0x26D6A3E8, 0x000F9344, 0x1DB88320,
        0xA005713C, 0xBDB26158, 0x9B6B51F4, 0x86DC4190,
        0xD6D930AC, 0xCB6E20C8, 0xEDB71064, 0xF0000000
    };

    unsigned int reg=0;

    // Preamble is not included in crc calculation
    for (std::vector<unsigned char>::iterator iter = eth_packet.begin()+8;
            iter != eth_packet.end();
            iter++)
    {
        unsigned char data = *iter;
        reg = (reg >> 4) ^ crc_table[(reg ^ (data >> 0)) & 0x0F];
        reg = (reg >> 4) ^ crc_table[(reg ^ (data >> 4)) & 0x0F];
    }
    //edited: fcs=frame check sequence 
    boost::array<unsigned char, 4> fcs;
    for (int n=0; n<4; n++)
    {
        fcs[n] = reg & 0xFF;
        reg >>= 8;
    }
    return fcs;
}


class CPhy_Transceiver
{
private: 
    
     enum Ports_num
    {
        RX_DATA  = 0,
        RX_CTRL  = 1,
        TX_DATA  = 2,
        TX_CTRL  = 3,
        MAX_PORT = 4
    };

     enum Recv_state
    {
        RECV_ST = 0,
        TRAN_ST = 1,
        WAIT_ST = 2
    };

  
    //recv side
    boost::asio::io_context* m_iocontext_sendrecv;
	std::vector<udp::socket*> m_socket_sendrecv;
	
	boost::array<unsigned char, 2048> m_udp_buf_recv;
    size_t m_udp_payload_len_recv;
    std::vector<unsigned char> m_eth_packet_recv;
    size_t m_remain_pkt_len_recv;
    size_t m_pkt_gap_cnt_recv;
    Recv_state m_receive_state;
	unsigned int m_current_recv_port;

    struct Ports_recv
    {
        mtiSignalIdT rxclk;
        mtiSignalIdT gmii_txd;
        mtiSignalIdT gmii_tx_dv;
    } m_ports_recv;

	std::vector<udp::endpoint*> m_remote_endpoint;

    // Ethernet packet
    std::vector<unsigned char> m_eth_packet_send;  

    // Internal buffer used for data conversion
    unsigned char* m_pbuffer_send;

    // Signal IDs
    struct Ports_send
    {
        mtiSignalIdT txclk;
        mtiSignalIdT gmii_rxd;
        mtiSignalIdT gmii_rx_en;
    } m_ports_send;

    unsigned char m_ipaddr[4];
    unsigned char m_macaddr[6];

public:
    CPhy_Transceiver(std::vector<int>& rx_dataport, std::vector<int>& rx_ctrlport, std::vector<int>& tx_dataport, std::vector<int>& tx_ctrlport, int * macaddr_int, int * ipaddr_int)
    {
        int numPorts = static_cast<int>(rx_dataport.size());

        for (int i=0; i<6; i++){
            m_macaddr[i] = macaddr_int[i] & 0xff;
        }
        for (int i=0; i<4; i++){
            m_ipaddr[i] = ipaddr_int[i] & 0xff;
        }

        m_iocontext_sendrecv = new boost::asio::io_context;
		
		// Create UDP Sockets
		for(int i = 0; i<numPorts; i++){
			udp::socket* m_socket_rx_d = new udp::socket(*m_iocontext_sendrecv, udp::endpoint(udp::v4(), rx_dataport[i]));
			udp::socket* m_socket_rx_c = new udp::socket(*m_iocontext_sendrecv, udp::endpoint(udp::v4(), rx_ctrlport[i]));
			udp::socket* m_socket_tx_d = new udp::socket(*m_iocontext_sendrecv, udp::endpoint(udp::v4(), tx_dataport[i]));
			udp::socket* m_socket_tx_c = new udp::socket(*m_iocontext_sendrecv, udp::endpoint(udp::v4(), tx_ctrlport[i]));
			
			m_socket_sendrecv.emplace_back(m_socket_rx_d);
			m_socket_sendrecv.emplace_back(m_socket_rx_c);
			m_socket_sendrecv.emplace_back(m_socket_tx_d);
			m_socket_sendrecv.emplace_back(m_socket_tx_c);
			
			m_remote_endpoint.emplace_back();
			m_remote_endpoint.emplace_back();
			m_remote_endpoint.emplace_back();
			m_remote_endpoint.emplace_back();
		}
        
        m_remain_pkt_len_recv = 0;
        m_pkt_gap_cnt_recv    = 0;
        m_receive_state       = RECV_ST;
        m_current_recv_port   = RX_DATA;

        // Get GMII interface recv signals
        m_ports_recv.rxclk       = mti_FindSignal(const_cast<char*>("gmii_rxclk"));
        m_ports_recv.gmii_txd    = mti_FindSignal(const_cast<char*>("gmii_rxd"));
        m_ports_recv.gmii_tx_dv  = mti_FindSignal(const_cast<char*>("gmii_rx_dv"));

        // Get GMII interface tx signals
        m_ports_send.txclk       = mti_FindSignal(const_cast<char*>("gmii_txclk"));
        m_ports_send.gmii_rxd    = mti_FindSignal(const_cast<char*>("gmii_txd"));
        m_ports_send.gmii_rx_en  = mti_FindSignal(const_cast<char*>("gmii_tx_en"));

        // Buffer for receiving gmii_rxd data
        m_pbuffer_send = new unsigned char[8];
    };

    ~CPhy_Transceiver()
    {
		for (auto sc : m_socket_sendrecv) {
			delete sc;
		}
		m_socket_sendrecv.clear();

        delete m_iocontext_sendrecv;
        delete[] m_pbuffer_send;
		
		for (auto ep : m_remote_endpoint) {
			delete ep;
		}
		m_remote_endpoint.clear();
		m_eth_packet_recv.clear();
    };

    void drive_GmiiTxd(unsigned char val)
    {
        unsigned char val_out[8];
        unsigned char tmp = val;
        for(size_t m=8;m>0;m--)
        {
            val_out[m-1] = (tmp & 0x01) + 2; // std_logic '0' is represented by 2, '1' is represented by 3
            tmp >>= 1;
        }
        mti_SetSignalValue(m_ports_recv.gmii_txd,(long) val_out);
    };

    void receive(std::vector<unsigned int>& portsList, unsigned int totalPorts)
    {
        boost::system::error_code error;
        size_t avail_len = 0;
        mtiInt32T rxclk_value;
		unsigned int portNum;
        rxclk_value = mti_GetSignalValue( m_ports_recv.rxclk );

        if(rxclk_value != 3)
        { 	// std_logic '1'
            //Only perform receiving at the rising edge of the clock
            return;
        }

        // If there are remaining data in the buffer, flush out those data before receiving 
        // any new data
        switch(m_receive_state)
        {
            case RECV_ST:
                //No output data
                drive_GmiiTxd(0);
                mti_SetSignalValue(m_ports_recv.gmii_tx_dv,(long) 2);
                
				// Check if the data is available at one of the UDP ports
                avail_len = m_socket_sendrecv[m_current_recv_port]->available();

                if (avail_len > 0)
                {
                    // Receive UDP packet
                    udp::endpoint remote_endpoint;

                    m_udp_payload_len_recv = m_socket_sendrecv[m_current_recv_port]->receive_from(boost::asio::buffer(m_udp_buf_recv),
                                                                            remote_endpoint, 0, error);

                    // Save the remote endpoint
                    if (m_remote_endpoint[m_current_recv_port] == NULL)
                    {
                        //if receive is call first, assign the reply port
						mti_PrintFormatted(const_cast<char*>("Auto reply port num(%d):%u \n"),m_current_recv_port,remote_endpoint.port());

                        m_remote_endpoint[m_current_recv_port] = new udp::endpoint(remote_endpoint);
                    }
                    else if (remote_endpoint.port() != m_remote_endpoint[m_current_recv_port]->port())
                    {
                        // if new simulation with old filxtor, update the reply port
						mti_PrintFormatted(const_cast<char*>("Update Auto reply port num(%d):%u \n"),m_current_recv_port,remote_endpoint.port());

                        m_remote_endpoint[m_current_recv_port]->port(remote_endpoint.port());
                    }

                    // Convert UDP packet to Ethernet packet
					portNum = portsList[m_current_recv_port];
                    to_ethernet_packet(m_current_recv_port, portNum);
            
                    m_remain_pkt_len_recv = m_eth_packet_recv.size();
                    m_receive_state = TRAN_ST;
                    //mti_Break();            
                }
                else 
                {
                    m_remain_pkt_len_recv = 0;
                    m_receive_state = RECV_ST;
                }
				
				m_current_recv_port = (m_current_recv_port+1)%totalPorts;
				
                break;
            case TRAN_ST:
                if(m_remain_pkt_len_recv > 0)
                {   // Has output data
                    // Force '1' on gmii_tx_dv
                    mti_SetSignalValue(m_ports_recv.gmii_tx_dv,(long) 3);

                    unsigned char cdata = *m_eth_packet_recv.begin();
                    drive_GmiiTxd(cdata);
            
                    m_eth_packet_recv.erase(m_eth_packet_recv.begin());
                    m_remain_pkt_len_recv--;

                    if (m_remain_pkt_len_recv == 0)
                    { 
                        //There is a 32-clock-cycle guard between two consecutive packets
                        m_pkt_gap_cnt_recv = 32;
                        m_receive_state = WAIT_ST;
                    }  
                    else
                    {
                        m_receive_state = TRAN_ST;
                    }
                }
                else //should never happen
                { 
                    m_receive_state = WAIT_ST;
                }
                break;

            case WAIT_ST:
                //No output data
                drive_GmiiTxd(0);
                mti_SetSignalValue(m_ports_recv.gmii_tx_dv,(long) 2);
                //There is a 20-clock-cycle guard between two consecutive packets
                if(m_pkt_gap_cnt_recv > 0)
                {
                    m_pkt_gap_cnt_recv--;
                    m_receive_state = TRAN_ST;
                }
                else
                {
                    m_receive_state = RECV_ST;
                }
                break;

            default:
                break;
        } 
    };
    //function to convert UDP packet to Ethernet packet
    void to_ethernet_packet(unsigned int nSelectPort, unsigned int portNum)
    {
        //mti_PrintFormatted((char*)&"Found data on dest port:%u \n",destPort);
        //mti_PrintFormatted((char*)&"Remote port:%u \n",remotePort);

        const boost::array<unsigned char, 8> eth_preamble   = {{0x55,0x55,0x55,0x55,0x55,0x55,0x55,0xD5}};

        boost::array<unsigned char, 6> eth_dstmacaddr;
        eth_dstmacaddr[0] = m_macaddr[0]; //was 0x00
        eth_dstmacaddr[1] = m_macaddr[1]; //was 0x0a
        eth_dstmacaddr[2] = m_macaddr[2]; //was 0x35
        eth_dstmacaddr[3] = m_macaddr[3]; //was 0x02
        eth_dstmacaddr[4] = m_macaddr[4]; //was 0x21
        eth_dstmacaddr[5] = m_macaddr[5]; //was 0x8a

        const boost::array<unsigned char, 6> eth_srcmacaddr = {{0xaa,0xbb,0xcc,0xdd,0xee,0xff}}; //Fake src MAC address
        const boost::array<unsigned char, 2> eth_iptype     = {{0x08,0x00}}; //IP packet type
        
        boost::array<unsigned char,20> eth_etherheader;
        eth_etherheader[0] = 0x45;   // IPV4
        eth_etherheader[1] = 0x00;   // IPV4
        size_t ip_len = 20 + 8 + m_udp_payload_len_recv; // IP header length + UDP header length + udp payload length
        eth_etherheader[2] = static_cast<unsigned char>(ip_len/256); // IP packet length 
        eth_etherheader[3] = ip_len%256;
        eth_etherheader[4] = 0x00; // identification
        eth_etherheader[5] = 0x00; // identification
        eth_etherheader[6] = 0x00; // flag & fragment
        eth_etherheader[7] = 0x00; // flag & fragment
        eth_etherheader[8] = 0x80; // time to live
        eth_etherheader[9] = 0x11; // udp protocol
        eth_etherheader[10] = 0x00; // header checksum, assign 0 as temporary value
        eth_etherheader[11] = 0x00; // header checksum, assign 0 as temporary value
       
        //Source IP Address
        eth_etherheader[12] = m_ipaddr[0]; // 192;
        eth_etherheader[13] = m_ipaddr[1]; // 168;
        eth_etherheader[14] = m_ipaddr[2]; // 0;
        eth_etherheader[15] = 1;

        //Dest IP Address
        eth_etherheader[16] = m_ipaddr[0]; // 192;
        eth_etherheader[17] = m_ipaddr[1]; // 168;
        eth_etherheader[18] = m_ipaddr[2]; // 0;
        eth_etherheader[19] = m_ipaddr[3]; // 2;
      
        //Calculate checksum
        unsigned int checksumtmp = 0;
        for(size_t ii=0;ii<20;ii++)
        {
            if(ii%2 == 1)
            {
                checksumtmp += (eth_etherheader[ii] & 0xff);
            }
            else
            {
                checksumtmp += 256*( eth_etherheader[ii] & 0xff );
            }
        }
        checksumtmp = (checksumtmp + checksumtmp/65536)%65536;
        eth_etherheader[10] = (checksumtmp / 256)^0xff;
        eth_etherheader[11] = (checksumtmp % 256)^0xff;

        //mti_PrintFormatted(const_cast<char*>("to_eth_pckt:portNum    : %d\n"), portNum);
        //mti_PrintFormatted(const_cast<char*>("to_eth_pckt:nSelectPort: %d\n"), nSelectPort);

        // Calculate UDP header
        boost::array<unsigned char,8> eth_udpheader;
        eth_udpheader[0] = 0x00; //Source port number
        eth_udpheader[1] = (unsigned char)nSelectPort + (unsigned char)portNum; //Source port number
        eth_udpheader[2] = 0x00; //Destination port number
        eth_udpheader[3] = (unsigned char)nSelectPort + (unsigned char)portNum; //Destination port number
        size_t udp_packet_len = 8 + m_udp_payload_len_recv;
        eth_udpheader[4] = static_cast<unsigned char>(udp_packet_len/256); // UDP packet length
        eth_udpheader[5] = udp_packet_len%256; // UDP packet length
        eth_udpheader[6] = 0; // UDP checksum checking is optional
        eth_udpheader[7] = 0; // UDP checksum checking is optional
        
        //clear all content of Ethernet packet
        m_eth_packet_recv.clear(); 
        //Concatenate all parts of Ethernet packet together
        m_eth_packet_recv.insert(m_eth_packet_recv.end(),eth_preamble.begin(),eth_preamble.end());
        m_eth_packet_recv.insert(m_eth_packet_recv.end(),eth_dstmacaddr.begin(),eth_dstmacaddr.end());
        m_eth_packet_recv.insert(m_eth_packet_recv.end(),eth_srcmacaddr.begin(),eth_srcmacaddr.end());
        m_eth_packet_recv.insert(m_eth_packet_recv.end(),eth_iptype.begin(),eth_iptype.end());
        m_eth_packet_recv.insert(m_eth_packet_recv.end(),eth_etherheader.begin(),eth_etherheader.end());
        m_eth_packet_recv.insert(m_eth_packet_recv.end(),eth_udpheader.begin(),eth_udpheader.end());
        m_eth_packet_recv.insert(m_eth_packet_recv.end(),m_udp_buf_recv.begin(),m_udp_buf_recv.begin() + m_udp_payload_len_recv);

        // Pad the packet to 64 bytes with zeros if necessary
        // The packet size doesn't include 8-byte preamble, but includes 4 byte fcs
        while (m_eth_packet_recv.size() < 68)
        {
            m_eth_packet_recv.insert(m_eth_packet_recv.end(),static_cast<unsigned char>(0));
        }
        // Calculate CRC of the packet
        boost::array<unsigned char,4> fcs = crc(m_eth_packet_recv);
        //Insert CRC at the end
        m_eth_packet_recv.insert(m_eth_packet_recv.end(),fcs.begin(),fcs.end());
    };

    // Convert buffer obtained from mti_GetSignalValue to integer
    uint8_t buf2integer(unsigned char* buffer)
    {
        uint8_t r = 0;
        for(int m=0;m<8;m++)
        {
            r <<= 1;
            if( buffer[m] == 3 ) 
            {
                r++;
            }
        }
        return r;
    };

    void transmit(unsigned int totalPorts)
    {
        mtiInt32T txclk_value;
        txclk_value = mti_GetSignalValue( m_ports_send.txclk );

        if(txclk_value != 3)
        {   // std_logic '1'
            //Only active at the rising edge of the clock
            return;
        }

        mtiInt32T gmii_rx_en_value;
        gmii_rx_en_value = mti_GetSignalValue( m_ports_send.gmii_rx_en );
		
		//Detect if gmii_rx_en is '1'
        if(gmii_rx_en_value == 3)
        {
            mti_GetArraySignalValue(m_ports_send.gmii_rxd, m_pbuffer_send);
            unsigned char data = buf2integer(m_pbuffer_send);
            m_eth_packet_send.push_back(data);

        }        
        else if( m_eth_packet_send.size() > 0 )
        { 	// Has data to transmit   
			
			size_t udp_payload_len = m_eth_packet_send[46]*256 + m_eth_packet_send[47] - 8;
            std::vector<unsigned char> udp_data(m_eth_packet_send.begin()+50, m_eth_packet_send.begin()+ 50 + udp_payload_len);
            std::vector<unsigned char> udp_port(m_eth_packet_send.begin()+44, m_eth_packet_send.begin()+46);
			
			//0 16 32 48 64 80  96  112
            //nSelectPorts:
			//0  4  8 12 16 20  24   28
            //udp_port[1]: USE THESE VALUES AS PORTS IN THE MAC HUB FOR CHANNEL ESTABLISHEMT
			//0 20 40 60 80 100 120 140
			signed int nSelectPort = udp_port[1]/5;
            //mti_PrintFormatted(const_cast<char*>("transmit:nSelectPort: %d\n"),nSelectPort);
            //mti_PrintFormatted(const_cast<char*>("transmit:udp_port[1]: %d\n"),udp_port[1]);

            if(nSelectPort < 0 || nSelectPort >= (int)totalPorts)
            {
				mti_PrintFormatted(const_cast<char*>("Error: Did not find matching reply port num: %d, abort sending. \n"),nSelectPort);
            }
            else
            {
                if(m_remote_endpoint[nSelectPort] != NULL)
                {
                    //mti_PrintFormatted((char*)&"Reply on port :%u \n",m_remote_endpoint[nSelectPort]->port());
                    m_socket_sendrecv[nSelectPort]->send_to(boost::asio::buffer(udp_data), *m_remote_endpoint[nSelectPort]);
                }
                else
                {
					mti_PrintFormatted(const_cast<char*>("Reply on port num(%u) is NULL \n"),nSelectPort);
                }
            }            

            m_eth_packet_send.clear(); // Clear UDP data

            //mti_Break();
        }
    };
};

// struct which contains info to be passed into receive and transmit methods of the transceiver
typedef struct {
		std::vector<unsigned int> portID;
		unsigned int totalPortsID;
} portsInfoT;

// Global pointer to the state
CPhy_Transceiver* pxtransceiver;

static void rxclk_cb(void* param)
{
	portsInfoT* portsInfo = new portsInfoT(*(portsInfoT*) param);
	pxtransceiver->receive(portsInfo->portID, portsInfo->totalPortsID);
	
	// Free the memory allocated by the new operator
    delete portsInfo;
}

static void txclk_cb(void* param)
{
	portsInfoT* portsInfo = new portsInfoT(*(portsInfoT*) param);
	pxtransceiver->transmit(portsInfo->totalPortsID);
	
	// Free the memory allocated by the new operator
    delete portsInfo;
}

// Make this function visible by ModelSim
// This will override the behavior of -fvisibility=hidden switch for GCC
#ifndef _WIN32
#pragma GCC visibility push(default)
extern "C" void filxtor_init(
  mtiRegionIdT       region,
  unsigned char      *param,
  mtiInterfaceListT *generics,
  mtiInterfaceListT *ports);
#pragma GCC visibility pop
#endif

extern "C" void filxtor_init(
  mtiRegionIdT       region,
  unsigned char      *param,
  mtiInterfaceListT  *generics,
  mtiInterfaceListT  *ports
)
{
	mti_PrintFormatted(const_cast<char*>("Input Parameter: %s\n"),param);

    std::vector<int> rx_dataport;
	std::vector<int> rx_ctrlport;
	std::vector<int> tx_dataport;
	std::vector<int> tx_ctrlport;
	int value;
    int macaddr[6];
    int ipaddr[4];
	std::vector<unsigned int> portsToUse;
	unsigned int portNumToUse;
	int count = 0, count_rxd = 0, count_rxc = 0, count_txd = 0, count_txc = 0;
	std::vector<char> portschar_vec;
	char *token;
	mtiProcessIdT txproc,rxproc;
	
    char c;
    while (sscanf((const char*)param, "%c", &c) == 1) {
		if (c == ' ') {
			param++;
			break;
		}
		portschar_vec.push_back(c);
		param++;matlab\test\tools\edatools\filxtor\filxtor.cpp
    }
	
	sscanf((const char*)param,"%x-%x-%x-%x-%x-%x,%d.%d.%d.%d",
            &macaddr[0], &macaddr[1], &macaddr[2], 
            &macaddr[3], &macaddr[4], &macaddr[5],
            &ipaddr[0], &ipaddr[1], &ipaddr[2], &ipaddr[3]);
	
	char* portschar = portschar_vec.data();
	
	token = strtok(reinterpret_cast<char*>(portschar), ",");
	
	while (token != NULL) {
		// Extract the port details and assign them to the corresponding arrays.
		if (count%4 == 0) {
            mti_PrintMessage(const_cast<char*>("in case 0\n"));
			
			if (std::sscanf(token, "%d", &value) == 1) {
				rx_dataport.push_back(value);
			}
			count_rxd++;
			count++;
		}
		else if (count%4 == 1) {
            mti_PrintMessage(const_cast<char*>("in case 1\n"));
			if (std::sscanf(token, "%d", &value) == 1) {
				rx_ctrlport.push_back(value);
			}
			count_rxc++;
			count++;
		}
		else if (count%4 == 2) {
            mti_PrintMessage(const_cast<char*>("in case 2\n"));
			if (std::sscanf(token, "%d", &value) == 1) {
				tx_dataport.push_back(value);
			}
			count_txd++;
			count++;
		}
		else if (count%4 == 3) {
            mti_PrintMessage(const_cast<char*>("in case 3\n"));
			if (std::sscanf(token, "%d", &value) == 1) {
				tx_ctrlport.push_back(value);
			}
			count_txc++;
			count++;
		}
		else {
			// not possible to reach here since MAX_PORT = 4;
            mti_PrintMessage(const_cast<char*>("in case NOT POSSIBLE\n"));
		}

		// get the next token
		token = strtok(NULL, ",");
	}
	
	mti_PrintFormatted(const_cast<char*>("FPGA HW MAC Address : %x-%x-%x-%x-%x-%x\n"),
											macaddr[0], macaddr[1], macaddr[2], 
											macaddr[3], macaddr[4], macaddr[5]);
	mti_PrintFormatted(const_cast<char*>("FPGA HW IP Address  : %d.%d.%d.%d\n"),
											ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
				
	mtiSignalIdT rxclk = mti_FindSignal(const_cast<char*>("gmii_rxclk"));
	mtiSignalIdT txclk = mti_FindSignal(const_cast<char*>("gmii_txclk"));
	
	// create the transceiver object
	pxtransceiver  = new CPhy_Transceiver(rx_dataport, rx_ctrlport, tx_dataport, tx_ctrlport,macaddr,ipaddr);
	
	for(int j = 0; j <= count/4-1; j++) {
		mti_PrintFormatted(const_cast<char*>("FPGA rx data port (%d)   : %d\n"),j, rx_dataport[j]);
		mti_PrintFormatted(const_cast<char*>("FPGA rx control port (%d): %d\n"),j, rx_ctrlport[j]);
		mti_PrintFormatted(const_cast<char*>("FPGA tx data port (%d)   : %d\n"),j, tx_dataport[j]);
		mti_PrintFormatted(const_cast<char*>("FPGA tx control port (%d): %d\n"),j, tx_ctrlport[j]);
	
		// Fetch the port number
		if (j == 0){
			portNumToUse = 0;
		}
		else {
			portNumToUse = portNumToUse + 16;
		}
		
		// push the value 4 times for rx, tx data and control ports respectively
		portsToUse.push_back(portNumToUse);
		portsToUse.push_back(portNumToUse);
		portsToUse.push_back(portNumToUse);
		portsToUse.push_back(portNumToUse);
	}
	
	// To avoid unexpected EOF error create separate
	portsInfoT* inst    = new portsInfoT;
	inst->portID        = portsToUse;
	inst->totalPortsID  = count;
	
	// Schedule callback functions
	rxproc = mti_CreateProcess(const_cast<char*>("p1"), rxclk_cb, inst);
	txproc = mti_CreateProcess(const_cast<char*>("p2"), txclk_cb, inst);
	
	mti_Sensitize(rxproc, rxclk, MTI_EVENT);
	mti_Sensitize(txproc, txclk, MTI_EVENT);
}