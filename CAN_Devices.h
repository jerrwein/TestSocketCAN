#define IC650_NODE_ID			(0x0A)

#define IC650_PRE_OPERATIONAL		(0x7F)
#define IC650_STOPPED			(0x04)
#define IC650_OPERATIONAL		(0x05)
#define IC650_NMT_START			(0x01)
#define IC650_NMT_STOP			(0x02)
#define IC650_NMT_PRE_OP		(0x80)
#define IC650_NMT_RESET_NODE		(0x81)
#define IC650_NMT_RESET_COMM		(0x82)

#define TX_NMT_COB_ID			(0x000)
#define IC650_TX_HEARTBEAT_COB_ID	(0x701)
#define IC650_TX_SDO_COB_ID		(0x60A)
#define IC650_RX_HEARTBEAT_COB_ID	(0x70A)
#define IC650_RX_SDO_COB_ID		(0x58A)
#define IC650_SDO_RD_CMD 		(0x40)
#define IC650_SDO_SEG_RD_CMD 		(0x60)
#define IC650_SDO_SEG_RD_TOGGLE_RESP	(0x00)
//#define IC650_SDO_SEG_RD_TOGGLE_1_RESP	(0x10)
#define IC650_SDO_MANUF_SW_VERSION	(0x100A)
#define IC650_SDO_IDENTITY		(0x1018)
#define IC650_SDO_IDENTITY_SUB_INDX_0	(0)
#define IC650_SDO_IDENTITY_SUB_INDX_1	(1)
#define IC650_SDO_IDENTITY_SUB_INDX_2	(2)
#define IC650_SDO_CHARGER_TEMPERATURE	(0x2050)
#define IC650_SDO_AC_VOLT_X16		(0x2200)
#define IC650_SDO_DC_VOLT_Q8P8		(0x2101)
#define IC650_SDO_EXTENDED_STATUS	(0x2006)
#define IC650_SDO_CHARGER_STATUS	(0x6001)

#define IC650_PDO1_COB_ID		(0x28A)
#define IC650_PDO2_COB_ID		(0x38A)
#define IC650_PDO3_COB_ID		(0x48A)


#define IC650_CMD_EXPED_TRANSFER_BIT	(0x02)
#define IC650_CMD_SERVER_CMD_SPEC_MASK	(0xE0)
#define IC650_CMD_SEG_TOGGLE_MASK	(0x10)
#define IC650_CMD_SEG_UNUSED_BYTES_MASK	(0x0E)
#define IC650_CMD_SEG_LAST_FRAME_MASK	(0x01)

typedef enum CAN_NODE_STATE
{
	NODE_PRE_OPERATIONAL,
	NODE_OPERATIONAL,
	NODE_STOPPED,
	NODE_OFF_LINE,
//	===============
	NODE_NUM_STATES
} CAN_NODE_STATE;

typedef struct CAN_NODE_STATUS
{
	CAN_NODE_STATE 		NodeState;
	unsigned long long	usec_last_NMT;
} CAN_NODE_STATUS;

//typedef enum SEG_TRANSFER_PHASE
//{
//	SEG_INITIAL_RESP,
//	SEG_TOG_0_FRAME,
//	SEG_TOG_1_FRAME,
//	SEG_LAST_FRAME,
//	===============
//	SEG_NUM_STATES
//} SEG_TRANSFER_PHASE;

typedef struct SEG_TRANSFER_INFO
{
//	SEG_TRANSFER_PHASE	TransferPhase;
	uint16_t		primary_index;
	uint8_t			sub_index;
	uint8_t			u8_toggle_state;
	uint8_t			data[100];
	uint32_t		u32_BytesExpected;
	uint32_t		u32_BytesTaken;
} SEG_TRANSFER_INFO;


int IC650_SendHeartbeat(int CAN_sock);
int IC650_SendOperationalNMT(int CAN_sock);
int IC650_SendReadSDO (uint16_t prim_index, uint8_t sub_index,int CAN_sock, bool bVerbose);
int IC650_SendSegmentedReadSDO (uint8_t u8_tog, int CAN_sock, bool bVerbose);
int IC650_ProcessRecvdFrame(CAN_DATA *pCanData, int CAN_sock, bool bVerbose);
int IC650_ProcessRecvdHeartbeat (CAN_DATA *pCanData, int CAN_sock);
int IC650_ProcessRecvdSDO (CAN_DATA *pCanData);
int IC650_ProcessRecvdSegmentedSDO (CAN_DATA *pCanData, SEG_TRANSFER_INFO *pSegInfo, bool bVerbose);
int IC650_ProcessRecvdPDO1 (CAN_DATA *pCanData);
int IC650_ProcessRecvdPDO2 (CAN_DATA *pCanData);
int IC650_ProcessRecvdPDO3 (CAN_DATA *pCanData);


