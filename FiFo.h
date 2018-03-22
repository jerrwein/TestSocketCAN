 
void	FIFO_RxInitialize(void);
int 	FIFO_RxIsDataAvailable(void);
int 	FIFO_RxHowManyElementsAvailable(void);
int 	FIFO_RxInsertElement(CAN_DATA *pCanData);
int 	FIFO_RxInsertData(CAN_DATA CanData[], int nLen);
int 	FIFO_RxGetElement(CAN_DATA *pCanData);
