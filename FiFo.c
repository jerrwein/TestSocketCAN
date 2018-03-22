#include <linux/can.h>
#include <linux/can/raw.h>

#include "CAN_Structures.h"
#include "FiFo.h"


#define RX_FIFO_SIZE 10

static CAN_RX_FIFO  CAN_RxDataFifo;

void FIFO_RxInitialize(void)
{
	short nIndex;

	CAN_RxDataFifo.nIndexNextIn = 0;
	CAN_RxDataFifo.nIndexNextOut = 0;
	for (nIndex=0; nIndex<RX_FIFO_SIZE; nIndex++)
	{
		CAN_RxDataFifo.CanData[nIndex].CanFrame.can_id = 0;
		CAN_RxDataFifo.CanData[nIndex].usec_delta = 0ll;
	}
	CAN_RxDataFifo.nStatus = 0;
}

int FIFO_RxIsDataAvailable(void)
{
    if (CAN_RxDataFifo.nIndexNextIn != CAN_RxDataFifo.nIndexNextOut)
        return 1;
    else
        return 0;
}

int FIFO_RxHowManyElementsAvailable(void)
{
    if (CAN_RxDataFifo.nIndexNextOut <= CAN_RxDataFifo.nIndexNextIn)
        return (CAN_RxDataFifo.nIndexNextIn - CAN_RxDataFifo.nIndexNextOut);
    else
        return (RX_FIFO_SIZE - (CAN_RxDataFifo.nIndexNextOut - CAN_RxDataFifo.nIndexNextIn));
}

int FIFO_RxInsertElement(CAN_DATA *pCanData)
{
    short nIndexNextIn;
  
    /* Are we about to crush existing data? */
    nIndexNextIn = (CAN_RxDataFifo.nIndexNextIn + 1) % RX_FIFO_SIZE;
    if (nIndexNextIn != CAN_RxDataFifo.nIndexNextOut)
    {
        CAN_RxDataFifo.CanData[CAN_RxDataFifo.nIndexNextIn] = *pCanData;
        CAN_RxDataFifo.nIndexNextIn = (unsigned char)nIndexNextIn;
        return 1;
    }
    else
    {
        /* Fifo fill failure status bit */
       // FIFO_TxCommStatus |= 0x01;
        return 0;
    }
}

int FIFO_RxInsertData(CAN_DATA CanData[], int nLen)
{
    int n, nRes;
    for (n=0; n<nLen; n++)
    {
        nRes = FIFO_RxInsertElement (&CanData[n]);
        if (1 != nRes)
        {
            /* Fifo fill failure status bit */
	// IFO_TxCommStatus |= 0x01;
            return n;
        }
    }
    return nLen;
}

int FIFO_RxGetElement(CAN_DATA *pCanData)
{
    if (CAN_RxDataFifo.nIndexNextIn != CAN_RxDataFifo.nIndexNextOut)
    {
        *pCanData = CAN_RxDataFifo.CanData[CAN_RxDataFifo.nIndexNextOut];
        CAN_RxDataFifo.nIndexNextOut = (unsigned char)((CAN_RxDataFifo.nIndexNextOut+1) % RX_FIFO_SIZE);
        return 1;
    }
    else
    {
        /* Nothing to give */
        return 0;
    }
}
