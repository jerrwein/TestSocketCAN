#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>
#include <stdbool.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <pthread.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <errno.h>


#define INTER_FRAME_DELAY_US (1000)

 

#include "CAN_Structures.h"
#include "FiFo.h"
#include "CAN_Devices.h"


static uint8_t 	rx_thread_shutdown;
static uint8_t	processing_thread_shutdown;

static CAN_NODE_STATUS IC650_Node_Status;


void *rx_thread (void *pArg)
{
	CAN_DATA		CanData;
	int 			cansock;
	int			nBytesRcvd, nRes;
	struct can_frame 	rxmsg;
	long			msgsRcvd = 0;

	//-----------------------------------------
	// initialize our message-header structure
	//-----------------------------------------
//	struct msghdr	mymsghdr = { 0 };
	struct msghdr	mymsghdr;
	struct msghdr	*msgp = &mymsghdr;
	struct cmsghdr	*cmsg;
	struct iovec	my_iov;
	unsigned char	tm_cbuf[40] = { 0 };
	int		tm_cbuf_len = sizeof(tm_cbuf); 

	IC650_Node_Status.NodeState = NODE_PRE_OPERATIONAL;

	cansock = *((int *)pArg); 

	printf("rx_thread() starting.\n");     

	/* set socket to non-blocking i/o */ 
	int flags = fcntl (cansock, F_GETFL, 0);
   	if (flags < 0)
	{
		printf("error getting flags\n");
		return NULL;
	}
 
	flags = flags | O_NONBLOCK;
	nRes = fcntl (cansock, F_SETFL, flags);
	if (0 != nRes)
	{
		printf("Non-Blocking mode NOT successful!\n");  
		return NULL;
	}

	// Enable timestamps
	int opt_val = 1;
	int opt_len = sizeof(opt_val);
	nRes = setsockopt (cansock, SOL_SOCKET, SO_TIMESTAMP, &opt_val, opt_len);
	if (nRes < 0)
	{
		printf("SO_TIMESTAMP mode NOT successful!\n");  
		return NULL;
	}

	// Get current time
	struct timeval	tv_base, tv_recv;
	if ( gettimeofday( &tv_base, NULL ) < 0 )
	{
		printf("Get base time NOT successful!\n");  
		return NULL;
	}
	unsigned long long	stamp_base, stamp_recv;
	unsigned long long	stamp_delta;
 
	stamp_base = tv_base.tv_sec * 1000000LL + tv_base.tv_usec;


	// Operate on the currently running thread.
     	pthread_t this_thread = pthread_self();

	// struct sched_param is used to store the scheduling priority
     	struct sched_param params;
 
     	// Set the priority to the maximum.
     	params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	
	printf ("Trying to set thread realtime priority = %d\n", params.sched_priority);
	nRes = pthread_setschedparam (this_thread, SCHED_FIFO, &params);
     	if (nRes != 0)
	{
         	// Print the error
         	printf ("Unsuccessful in setting thread realtime priority\n");
		printf("Rx thread: exitting.\n");
		pthread_exit((void *)149);
	}

 	// Now verify the change in thread priority
     	int policy = 0;
     	nRes = pthread_getschedparam(this_thread, &policy, &params);
     	if (nRes != 0)
	{
		printf("Couldn't retrieve real-time scheduling parameters\n");
         	printf("Rx thread: exitting.\n");
		pthread_exit((void *)153);
     	}
 
     	// Check the correct policy was applied
     	if (policy != SCHED_FIFO) 
	{
  		printf("Scheduling is NOT SCHED_FIFO!\n");
	}
	else
	{
         	printf("Scheduling is SCHED_FIFO\n");
     	}
 
     	// Print thread scheduling priority
     	printf("Thread priority is %d\n", params.sched_priority); 

	my_iov.iov_base = &rxmsg;
	my_iov.iov_len = sizeof(rxmsg);

	mymsghdr.msg_name	= NULL;
	mymsghdr.msg_namelen	= 0;
	mymsghdr.msg_iov	= &my_iov;
	mymsghdr.msg_iovlen	= 1;
	// mymsghdr.msg_control	= NULL;
	mymsghdr.msg_control	= tm_cbuf;
	// mymsghdr.msg_controllen	= 0;
	mymsghdr.msg_controllen	= tm_cbuf_len;
	mymsghdr.msg_flags	= 0;

	while (!rx_thread_shutdown) 
	{   
	//	nBytesRcvd = read (cansock, &rxmsg, sizeof(rxmsg));
		nBytesRcvd = recvmsg (cansock, &mymsghdr, MSG_DONTWAIT);
		if (0 < nBytesRcvd)
		{
			if (nBytesRcvd != sizeof(struct can_frame))
			{
				printf("Rx(1) message(%ld) bytes: %d\n", msgsRcvd, nBytesRcvd);
			}
			else
			{
			// aaa	printf("Rx(2) message(%ld) bytes: %d\n", msgsRcvd, nBytesRcvd);
			}

 
			//-------------------------------------------------------
			// Apply system macros for accessing the ancilliary data
			//-------------------------------------------------------
			for (cmsg=CMSG_FIRSTHDR(msgp); cmsg != NULL; cmsg = CMSG_NXTHDR(msgp,cmsg))
			{
				if ((cmsg->cmsg_level == SOL_SOCKET) && (cmsg->cmsg_type == SO_TIMESTAMP))
				{
				 	memcpy (&tv_recv, CMSG_DATA(cmsg), sizeof(tv_recv));

					stamp_recv = tv_recv.tv_sec * 1000000LL + tv_recv.tv_usec;
					stamp_delta = stamp_recv - stamp_base;
				// aaa	printf("Rx delta = %lld\n", stamp_delta);  
					break;
				}
				else
				{
					stamp_delta = 0ll;
				}
			}
			if (cmsg == NULL) 
			{
        			/* Error: SO_TIMESTAMP not enabled or similar */
				printf ("No SO_TIMESTAMP\n");
			}

			CanData.CanFrame = rxmsg;
			CanData.usec_delta = stamp_delta;

			nRes = FIFO_RxInsertElement(&CanData);
			msgsRcvd++;
			if (0 == nRes)
			{
				printf("Rx message(%ld) %d NOT queued!\n", msgsRcvd, nBytesRcvd);  
			}
	 	}
		usleep(2000);
	};

	printf("Rx thread: shut down.\n");
	pthread_exit((void *)272);
}


void *processing_thread (void *pArg)
{
	CAN_DATA		CanData;
	int 			cansock;
	int			nRes;
//	struct can_frame 	rxmsg;
	long			msgsRcvd = 0;
	struct sched_param 	params;
	int			HB_Countdown = 100;

	cansock = *((int *)pArg);

	printf("processing_thread() starting.\n");

  	// Operate on the currently running thread.
     	pthread_t this_thread = pthread_self();

	// Set the priority to the maximum.
     	params.sched_priority = sched_get_priority_max(SCHED_FIFO);
	
	printf ("Trying to set thread realtime priority = %d\n", params.sched_priority);
	nRes = pthread_setschedparam (this_thread, SCHED_FIFO, &params);
     	if (nRes != 0)
	{
	      	// Print the error
         	printf ("Unsuccessful in setting thread realtime priority\n");
		printf("Processing thread: exiting.\n");
		pthread_exit((void *)249);
	}

 	// Now verify the change in thread priority
     	int policy = 0;
     	nRes = pthread_getschedparam(this_thread, &policy, &params);
     	if (nRes != 0)
	{
		printf("Couldn't retrieve real-time scheduling parameters\n");
         	printf("Processing thread: exiting.\n");
		pthread_exit((void *)253);
     	}
 
     	// Check the correct policy was applied
     	if (policy != SCHED_FIFO) 
	{
  		printf("Scheduling is NOT SCHED_FIFO!\n");
	}
	else
	{
         	printf("Scheduling is SCHED_FIFO\n");
     	}
 
     	// Print thread scheduling priority
     	printf("Processing thread priority is %d\n", params.sched_priority); 
 
	while (!processing_thread_shutdown) 
	{   
	//	printf("processing_thread...\n");
		while (FIFO_RxIsDataAvailable())
		{
			msgsRcvd++;
			nRes = 	FIFO_RxGetElement (&CanData);
			/* Update status whenever charger heartbeats occur */
			
			if ((IC650_PDO1_COB_ID != CanData.CanFrame.can_id) &&
			    (IC650_PDO2_COB_ID != CanData.CanFrame.can_id) &&
			    (IC650_PDO3_COB_ID != CanData.CanFrame.can_id))
			{
				IC650_ProcessRecvdFrame (&CanData, cansock, false /* verbose */);
			}
 		};
	 	usleep(1000);
		if (--HB_Countdown == 0)
		{
			if (0 != IC650_SendHeartbeat(cansock))
			{
				printf("** Error sending HB **\n");
			}
//			else
//			{
//				printf("** Sent HB **\n");
//			}
			HB_Countdown = 500;
		}
 	};

	printf("Processing thread: shut down.\n");
	pthread_exit((void *)395);
}


int main(int argc,char **argv)
{
	int 			cansock, nRes;
	struct sockaddr_can 	addr;
//	struct can_frame 	tx_frame;	//, rx_frame;
	struct ifreq 		ifr;
//	long			nRxCount = 0;
	pthread_t 		pRxThread;
	pthread_t 		pProcessingThread;
	long			msgsRcvd = 0;
	CAN_DATA		CanData;
//	int 			errsv;
	int 			k;
   	char 			user_input[100];
    
	char *ifname = argv[1];

	if (argc != 2)
	{
		printf("usage : sendCAN <CAN interface>\n");
		printf("ex. sendCAN can1\n");
		return 0;
	}
 
	if ((cansock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Error while opening socket");
		return -1;
	}

	strcpy (ifr.ifr_name, ifname);
	ioctl (cansock, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex; 

	if (bind(cansock, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
	{
		perror("Error in socket bind");
		return -2;
	}

	rx_thread_shutdown = 0;	
	nRes = pthread_create (&pRxThread, NULL, &rx_thread, (void *)(&cansock));
	if (nRes != 0)
	{
		printf("pthread_create(RX) FAILED!\n");
		return -3;
	}

//	processing_thread_shutdown = 0;
 //	nRes = pthread_create (&pProcessingThread, NULL, &processing_thread, (void *)(&cansock));
//	if (nRes != 0)
//	{
//		printf("pthread_create(PROC) FAILED!\n");
//		return -3;
//	}

	for (k=0; k<8; k++)
	{
		// Take a peek at the Rx FIFO
		while (FIFO_RxIsDataAvailable())
		{
			msgsRcvd++;
			nRes = 	FIFO_RxGetElement (&CanData);
			/* Update status whenever charger heartbeats occur */
			IC650_ProcessRecvdFrame (&CanData, cansock, false);
 		};
		 
		if (0 != IC650_SendHeartbeat(cansock))
		{
			printf("** Error sending HB **\n");
		}
 		usleep(INTER_FRAME_DELAY_US);
		
	//	if (0 != IC650_SendReadSDO (IC650_SDO_MANUF_SW_VERSION, 0 /* sub-index */, cansock, false))
	//	{
	//		printf("Error sending SDO(SDO_MANUF_SW)\n");
	//	}
 	//	usleep(INTER_FRAME_DELAY_US); 

		if (0 != IC650_SendReadSDO (IC650_SDO_IDENTITY, 2 /* sub-index */, cansock, false))
		{
			printf("Error sending SDO(SDO_IDENTITY)\n");
		}
 		usleep(INTER_FRAME_DELAY_US); 

 		if (0 != IC650_SendReadSDO (IC650_SDO_AC_VOLT_X16, 0 /* sub-index */, cansock, false))
		{
			printf("Error sending SDO(SDO_AC_VOLT_X16)\n");
		}	
 		usleep(INTER_FRAME_DELAY_US);
		if (0 != IC650_SendReadSDO (IC650_SDO_DC_VOLT_Q8P8, 0 /* sub-index */, cansock, false))
		{
			printf("Error sending SDO(DC_VOLT_Q8P8)\n");
		}	
 		usleep(INTER_FRAME_DELAY_US);
		if (0 != IC650_SendReadSDO (IC650_SDO_CHARGER_TEMPERATURE, 0 /* sub-index */, cansock, false))
		{
			printf("Error sending SDO(SDO_CHARGER_TEMPERATURE)\n");
		}	
 		usleep(INTER_FRAME_DELAY_US);
		if (0 != IC650_SendReadSDO (IC650_SDO_EXTENDED_STATUS, 0 /* sub-index */, cansock, false))
		{
			printf("Error sending SDO(SDO_EXTENDED_STATUS)\n");
		}	
  		usleep(INTER_FRAME_DELAY_US);
 	}


	processing_thread_shutdown = 0;
 	nRes = pthread_create (&pProcessingThread, NULL, &processing_thread, (void *)(&cansock));
	if (nRes != 0)
	{
		printf("pthread_create(PROC) FAILED!\n");
		return -3;
	}
	usleep(200000);


	bool bQuit = false;
	int user_val;
	do
	{
		printf ("1.) AC Voltage?  2.) DC  3.) Int. Temp? 4.) Ext. Status  5.) Ident.  6.) F/W  9.) Quit\n");
		fgets (user_input, 100, stdin);
		
		user_val = atoi(user_input);
	//	printf ("Your input: %s (%d)", user_input, user_val);
		switch (user_val)
		{
			case 1:
				if (0 != IC650_SendReadSDO (IC650_SDO_AC_VOLT_X16, 0 /* sub-index */, cansock, false))
				{
					printf("Error sending SDO(SDO_AC_VOLT_X16)\n");
				}	
				break;
			case 2:
				if (0 != IC650_SendReadSDO (IC650_SDO_DC_VOLT_Q8P8, 0 /* sub-index */, cansock, false))
				{
					printf("Error sending SDO(SDO_VOLT_Q8P8)\n");
				}	
				break;
			case 3:
				if (0 != IC650_SendReadSDO (IC650_SDO_CHARGER_TEMPERATURE, 0 /* sub-index */, cansock, false))
				{
					printf("Error sending SDO(SDO_CHARGER_TEMPERATURE)\n");
				}	
				break;
			case 4:
				if (0 != IC650_SendReadSDO (IC650_SDO_EXTENDED_STATUS, 0 /* sub-index */, cansock, false))
				{
					printf("Error sending SDO(SDO_EXTENDED_STATUS)\n");
				}
				break;
			case 5:
				if (0 != IC650_SendReadSDO (IC650_SDO_IDENTITY, 3 /* sub-index */, cansock, false))
				{
					printf("Error sending SDO(SDO_IDENTITY)\n");
				}	
				break;
			case 6:
				if (0 != IC650_SendReadSDO (IC650_SDO_MANUF_SW_VERSION, 0 /* sub-index */, cansock, false))
				{
					printf("Error sending SDO(SDO_MAN_SW)\n");
				}	
				break;
			default:
				bQuit = true;
		};
		usleep(50000);
	} while (!bQuit);

	printf("Shutting down Rx thread...\n");
	rx_thread_shutdown = 1;	

	void        *thread_ret;	
	nRes = pthread_join (pRxThread, &thread_ret);
    	if (nRes != 0)
	{
		printf("Error joining RX thread.\n");
       	}
	printf("rx_thread exit code %ld\n", (long)thread_ret);

	processing_thread_shutdown = 1;
	nRes = pthread_join (pProcessingThread, &thread_ret);
    	if (nRes != 0)
	{
		printf("Error joining PROC thread.\n");
       	}
	printf("processing_thread exit code %ld\n", (long)thread_ret);

	return 0;
}

 
 

int IC650_SendOperationalNMT(int CAN_sock)
{
	struct can_frame 	tx_frame;
	int 			nbytes;
	int			errsv;

	tx_frame.can_id  = TX_NMT_COB_ID;
	tx_frame.data[0] = IC650_NMT_START;
	tx_frame.data[1] = IC650_NODE_ID;
	tx_frame.can_dlc = 2;

	nbytes = write (CAN_sock, &tx_frame, sizeof(struct can_frame));
	errsv = errno;
	if (nbytes != sizeof(struct can_frame))
	{
		printf("BW: %d, LE #: %d (%s)\n", nbytes, errsv, strerror(errsv));
		return -1;
	}
	return 0;
}

int IC650_SendHeartbeat(int CAN_sock)
{
	struct can_frame 	tx_frame;
	int 			nbytes;
	int			errsv;

	tx_frame.can_id  = IC650_TX_HEARTBEAT_COB_ID;
	tx_frame.data[0] = IC650_OPERATIONAL;
	tx_frame.can_dlc = 1;

	nbytes = write (CAN_sock, &tx_frame, sizeof(struct can_frame));
	errsv = errno;
	if (nbytes != sizeof(struct can_frame))
	{
		printf("BW: %d, LE #: %d (%s)\n", nbytes, errsv, strerror(errsv));
		return -1;
	}
	return 0;
}

int IC650_SendReadSDO (uint16_t prim_index, uint8_t sub_index, int CAN_sock, bool bVerbose)
{
	struct can_frame 	tx_frame;
	int 			nbytes;
	int			err_last;

	tx_frame.can_id  = IC650_TX_SDO_COB_ID;
	tx_frame.data[0] = IC650_SDO_RD_CMD;
	tx_frame.data[1] = (uint8_t)(prim_index & 0x00ff);
	tx_frame.data[2] = (uint8_t)((prim_index & 0xff00) >> 8);
	tx_frame.data[3] = sub_index;
	tx_frame.can_dlc = 4;

	if (bVerbose)
	{
		int i;	
		printf("  Tx: ID=0x%03X, DLC=%d\n", tx_frame.can_id, tx_frame.can_dlc);
		printf("    ");
  		for (i =0; i<tx_frame.can_dlc; i++)
		{
  			printf("0x%02X ", tx_frame.data[i]);
		}
		printf("\n");
	}

	nbytes = write (CAN_sock, &tx_frame, sizeof(struct can_frame));
	err_last = errno;
	if (nbytes != sizeof(struct can_frame))
	{
		printf("BW: %d, LE #: %d (%s)\n", nbytes, err_last, strerror(err_last));
		return -1;
	}
	return 0;
}

int IC650_SendSegmentedReadSDO (uint8_t u8_tog, int CAN_sock, bool bVerbose)
{
	struct can_frame 	tx_frame;
	int 			nbytes;
	int			err_last;

	tx_frame.can_id  = IC650_TX_SDO_COB_ID;
	tx_frame.data[0] = IC650_SDO_SEG_RD_CMD | u8_tog;
	tx_frame.data[1] = 0x00;
	tx_frame.data[2] = 0x00;
	tx_frame.data[3] = 0x00;
	tx_frame.data[4] = 0x00;
	tx_frame.data[5] = 0x00;
	tx_frame.data[6] = 0x00;
	tx_frame.data[7] = 0x00;
	tx_frame.can_dlc = 1;

	if (bVerbose)
	{
		int i;	
		printf("  Tx: ID=0x%03X, DLC=%d\n", tx_frame.can_id, tx_frame.can_dlc);
		printf("    ");
  		for (i =0; i<tx_frame.can_dlc; i++)
		{
  			printf("0x%02X ", tx_frame.data[i]);
		}
		printf("\n");
	}

	nbytes = write (CAN_sock, &tx_frame, sizeof(struct can_frame));
	err_last = errno;
	if (nbytes != sizeof(struct can_frame))
	{
		printf("BW: %d, LE #: %d (%s)\n", nbytes, err_last, strerror(err_last));
		return -1;
	}
	else
	{
	//	printf("Segmented BW: %d\n", nbytes);
	}
	return 0;
}

int IC650_ProcessRecvdFrame(CAN_DATA *pCanData, int CAN_sock, bool bVerbose)
{
	static SEG_TRANSFER_INFO 	segXferInfo;
	int 				i;

	if (IC650_RX_SDO_COB_ID == pCanData->CanFrame.can_id)
	{
		if (bVerbose)
		{
			printf("  Rx: ID=0x%03X, DLC=%d \n", pCanData->CanFrame.can_id, pCanData->CanFrame.can_dlc);
			printf("    ");
			for (i =0; i<pCanData->CanFrame.can_dlc; i++)
			{
  				printf("0x%02X ", pCanData->CanFrame.data[i]);
			}
			printf("\n");
		}

		if (IC650_SDO_SEG_RD_TOGGLE_RESP == (pCanData->CanFrame.data[0] & IC650_CMD_SERVER_CMD_SPEC_MASK))
		{
			// Process Toggle-X Frame
			if (0 == IC650_ProcessRecvdSegmentedSDO (pCanData, &segXferInfo, false /* verbosity */))
			{
				if (IC650_CMD_SEG_LAST_FRAME_MASK & pCanData->CanFrame.data[0])
				{
					if (bVerbose)
					{
						printf("ProcessRecvdSegmentedSDO(SEG_TOG-0): bytes taken = %d\n", segXferInfo.u32_BytesTaken);
					}
					// Todo: Process segments
					if (segXferInfo.u32_BytesTaken != segXferInfo.u32_BytesExpected)
					{
						printf("Segmented Xfer - MISMATCH in bytes received, expected = %d\n", segXferInfo.u32_BytesExpected);
					}
					else
					{
						if (bVerbose)
						{
							printf("Segmented Xfer - LAST frame received\n");
							printf("Data: %s\n", segXferInfo.data);
						}
						if (IC650_SDO_MANUF_SW_VERSION == segXferInfo.primary_index)
						{						
							printf("P-SDO(SW-VER): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, segXferInfo.primary_index);
							printf("P-SDO(SW-VER): Version: %s\n", segXferInfo.data);
						}
					}
				}
				else
				{
					// Toggle polarity flag
					segXferInfo.u8_toggle_state ^= IC650_CMD_SEG_TOGGLE_MASK;
				//	printf("Segmented Xfer - Send TOG-(0x%02X) request\n", segXferInfo.u8_toggle_state);
					if (0 != IC650_SendSegmentedReadSDO (segXferInfo.u8_toggle_state, CAN_sock, bVerbose))
					{
						printf("Segmented Xfer TOG-X seg request failed!!!\n");
					}
				}
			}
		}
		else if (!(IC650_CMD_EXPED_TRANSFER_BIT & pCanData->CanFrame.data[0]))	/* 0x02 */
		{
			if (bVerbose)
			{
				printf("Segmented Xfer starting\n");
			}
			segXferInfo.primary_index = (((uint16_t)pCanData->CanFrame.data[2]) << 8)
			   			  + ((uint16_t)pCanData->CanFrame.data[1]);
			segXferInfo.sub_index = pCanData->CanFrame.data[3];
			segXferInfo.u32_BytesExpected = (((uint16_t)pCanData->CanFrame.data[7]) << 24)
						      + (((uint16_t)pCanData->CanFrame.data[6]) << 16)
						      + (((uint16_t)pCanData->CanFrame.data[5]) << 8)
						      + ((uint16_t)pCanData->CanFrame.data[4]);
			segXferInfo.u32_BytesTaken = 0;
			segXferInfo.u8_toggle_state = 0x00;
			// Clear receive buffer
			for (i=0; i<25; i++)
			{
				segXferInfo.data[i] = 0;
			}
			//pSegInfo->TransferPhase = SEG_TOG_0_FRAME;
			if (bVerbose)
			{
				printf("ProcessRecvdFrame(INITIAL_RESP): bytes expected = %d\n", segXferInfo.u32_BytesExpected);
			}
//			printf("Expedited Xfer - Send TOG-0 request\n");
			if (0 != IC650_SendSegmentedReadSDO (segXferInfo.u8_toggle_state, CAN_sock, bVerbose))
			{
				printf("Segmented Xfer 1st. seg request failed!!!\n");
			}
		}
		else
		{
			IC650_ProcessRecvdSDO (pCanData);
		}
	}
	else if (IC650_PDO1_COB_ID == pCanData->CanFrame.can_id)
	{
		IC650_ProcessRecvdPDO1 (pCanData);
	}
	else if (IC650_PDO2_COB_ID == pCanData->CanFrame.can_id)
	{
		IC650_ProcessRecvdPDO2 (pCanData);
	}
	else if (IC650_PDO3_COB_ID == pCanData->CanFrame.can_id)
	{
		IC650_ProcessRecvdPDO3 (pCanData);
	}
	else if (IC650_RX_HEARTBEAT_COB_ID == pCanData->CanFrame.can_id)
	{
		IC650_ProcessRecvdHeartbeat (pCanData, CAN_sock);
	}
  	else
	{
		printf("PF: What ??? ID: 0x%03X\n", pCanData->CanFrame.can_id);
	}
 	return 0;
}

int IC650_ProcessRecvdSegmentedSDO (CAN_DATA *pCanData, SEG_TRANSFER_INFO *pSegInfo, bool bVerbose)
{
	// uint32_t 	u32_val;
	int		n;

	uint8_t u8_unused = (IC650_CMD_SEG_UNUSED_BYTES_MASK & pCanData->CanFrame.data[0]) >> 1;

	if (bVerbose)
	{
		printf("ProcessRecvdSegmentedSDO(): bytes unused = %d\n", u8_unused);
	}
 
	// Check TOGGLE polarity
	if (pSegInfo->u8_toggle_state != (pCanData->CanFrame.data[0] & IC650_CMD_SEG_TOGGLE_MASK))
	{
		printf("ProcessRecvdSegmentedSDO(): Unexpected Toggle bit = 0x%02X\n", (pCanData->CanFrame.data[0] & IC650_CMD_SEG_TOGGLE_MASK));
		printf("ProcessRecvdSegmentedSDO(): Expected Toggle bit = 0x%02X\n", pSegInfo->u8_toggle_state);
		// ToDo: Cancel segment transfer
		return -1;
	}		
	for (n=0; n<(7-u8_unused); n++)
	{
		pSegInfo->data[pSegInfo->u32_BytesTaken+n] = pCanData->CanFrame.data[n+1];
	}
	pSegInfo->u32_BytesTaken += (7-u8_unused);

 	return 0;
 }

int IC650_ProcessRecvdHeartbeat (CAN_DATA *pCanData, int CAN_sock)
{
#if 0
	int i;
	printf("Rcvd_HB: ID=0x%x, DLC=%d, 0x%02X\n", pCanData->CanFrame.can_id, pCanData->CanFrame.can_dlc, pCanData->CanFrame.data[0]);
	for (i =0; i < pCanData->CanFrame.can_dlc; i++)
	{
		printf("0x%x ", pCanData->CanFrame.data[i]);
	}
	printf("\nRx delta = %lld.%06lld\n", pCanData->usec_delta/1000000ll, pCanData->usec_delta%1000000ll); 	
#endif	
	if ((IC650_PRE_OPERATIONAL == pCanData->CanFrame.data[0]) ||
	    (IC650_STOPPED == pCanData->CanFrame.data[0]))
	{
		if (NODE_PRE_OPERATIONAL == pCanData->CanFrame.data[0])
		{
			IC650_Node_Status.NodeState = NODE_PRE_OPERATIONAL;
		}		
		else if (NODE_STOPPED == pCanData->CanFrame.data[0])
		{
			IC650_Node_Status.NodeState = NODE_STOPPED;
		}
		if (0 != IC650_SendOperationalNMT(CAN_sock))
		{
			printf("IC650_ProcessRecvdHeartbeat(): ! FAILURE sending NMT\n");
		}
		else
		{
			printf("IC650_ProcessRecvdHeartbeat(OPERATIONAL): sent NMT\n");
		}
	}

	return 0;
}

int IC650_ProcessRecvdSDO (CAN_DATA *pCanData)
{
	float f32_val;
	uint16_t u16_val;
	uint32_t u32_val;
	uint16_t u8_val;
	uint16_t primary_index = (((uint16_t)pCanData->CanFrame.data[2]) << 8)
			       + ((uint16_t)pCanData->CanFrame.data[1]);

	if (IC650_SDO_RD_CMD == (0xf0 & pCanData->CanFrame.data[0]))
	{
		switch (primary_index)
		{
			case IC650_SDO_IDENTITY:
				printf("P-SDO(IDNT-3): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
				u32_val = (((uint16_t)pCanData->CanFrame.data[7]) << 24)
					+ (((uint16_t)pCanData->CanFrame.data[6]) << 16)
			       		+ (((uint16_t)pCanData->CanFrame.data[5]) << 8)
			       		+ ((uint16_t)pCanData->CanFrame.data[4]);
				printf("P-SDO(IDNT-3): Version: %08X\n", u32_val);
				break;
			case IC650_SDO_AC_VOLT_X16:
				printf("P-SDO(AC_X16): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
				u16_val = (((uint16_t)pCanData->CanFrame.data[5]) << 8)
			       		+ ((uint16_t)pCanData->CanFrame.data[4]);
			//	printf("P-SDO(AC_X16): AC: 0x%04X, AC/16 (%d.%02f)\n", u16_val, u16_val>>4, (float)(u16_val&0x000f)/16.0f);
				f32_val = (float)(u16_val) / 16.0f;
				printf("P-SDO(AC_X16): AC: %3.3f\n", f32_val);
				break;
			case IC650_SDO_DC_VOLT_Q8P8:
				printf("P-SDO(DC_VOLT_Q8.6): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
				u16_val = (((uint16_t)pCanData->CanFrame.data[5]) << 8)
			       		+ ((uint16_t)pCanData->CanFrame.data[4]);
			//	printf("P-SDO(AC_X16): AC: 0x%04X, AC/16 (%d.%02f)\n", u16_val, u16_val>>4, (float)(u16_val&0x000f)/16.0f);
				f32_val = (float)(u16_val) / 256.0f;
				printf("P-SDO(DC_Q8.8): DC: %3.3f\n", f32_val);
				break;
			case IC650_SDO_CHARGER_TEMPERATURE:
				printf("P-SDO(CHRG_TEMP): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
				u8_val = pCanData->CanFrame.data[4];
				printf("P-SDO(CHRG_TEMP): Charger Temperature: 0x%02X (%dd C.)\n", u8_val, u8_val);
				break;
			case IC650_SDO_EXTENDED_STATUS:
				printf("P-SDO(EXT_STS): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
				u16_val = (((uint16_t)pCanData->CanFrame.data[5]) << 8)
			       		+ ((uint16_t)pCanData->CanFrame.data[4]);
				printf("P-SDO(EXT_STS): AC Present: %d\n", (u16_val & 0x0010) ? 1 : 0);
				break;
			case IC650_SDO_MANUF_SW_VERSION:
				printf("P-SDO(MAN-SW): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
				u32_val = (((uint16_t)pCanData->CanFrame.data[7]) << 24)
					+ (((uint16_t)pCanData->CanFrame.data[6]) << 16)
			       		+ (((uint16_t)pCanData->CanFrame.data[5]) << 8)
			       		+ ((uint16_t)pCanData->CanFrame.data[4]);
				printf("P-SDO(MAN-SW): Version: %08X\n", u32_val);
				break;
			default:
				printf("P-SDO(---): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
		};
	}
	else
	{
		printf("PF(?): ID: 0x%03X, PrimIndx:%04X, D[0]: 0x%02X\n", pCanData->CanFrame.can_id, primary_index, pCanData->CanFrame.data[0]);
	}
	return 0;
}

int IC650_ProcessRecvdPDO1 (CAN_DATA *pCanData)
{
	static unsigned long long	usec_last_frame = 0ll;
	unsigned long long		usec_delta;

	usec_delta = pCanData->usec_delta - usec_last_frame;
	usec_last_frame = pCanData->usec_delta;

	printf("Proc-PDO1(): ID: 0x%03X, delta = %lld.%06lld\n", pCanData->CanFrame.can_id, usec_delta/1000000ll, usec_delta%1000000ll);
	return 0;
}

int IC650_ProcessRecvdPDO2 (CAN_DATA *pCanData)
{
	static unsigned long long	usec_last_frame = 0ll;
	unsigned long long		usec_delta;

	usec_delta = pCanData->usec_delta - usec_last_frame;
	usec_last_frame = pCanData->usec_delta;

	printf("Proc-PDO2(): ID: 0x%03X, delta = %lld.%06lld\n", pCanData->CanFrame.can_id, usec_delta/1000000ll, usec_delta%1000000ll);
	return 0;
}
int IC650_ProcessRecvdPDO3 (CAN_DATA *pCanData)
{
	static unsigned long long	usec_last_frame = 0ll;
	unsigned long long		usec_delta;

	usec_delta = pCanData->usec_delta - usec_last_frame;
	usec_last_frame = pCanData->usec_delta;

	//printf("\nRx delta = %lld.%06lld\n", pCanData->usec_delta/1000000ll, pCanData->usec_delta%1000000ll); 

	printf("Proc-PDO3(): ID: 0x%03X, delta = %lld.%06lld\n", pCanData->CanFrame.can_id, usec_delta/1000000ll, usec_delta%1000000ll);
	return 0;
}
