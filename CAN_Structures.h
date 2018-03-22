#define RX_FIFO_SIZE 10

typedef struct
{
	struct can_frame 	CanFrame;
	unsigned long long	usec_delta;
//	unsigned long 		tv_sec;
//	unsigned long		tv_usec;
} CAN_DATA;

typedef struct
{
	unsigned int	nStatus;
	unsigned char	nIndexNextIn;
	unsigned char	nIndexNextOut;
	CAN_DATA 	CanData[RX_FIFO_SIZE];
} CAN_RX_FIFO;
