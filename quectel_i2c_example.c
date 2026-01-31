#define IOP_LF_DATA 0x0A // <LF>
#define IOP_CR_DATA 0x0D // <CR>
#define IOP_START_DBG 0x23 // debug log start char „#‟
#define IOP_START_NMEA 0x24// NMEA start char „$‟
#define IOP_START_HBD1 'H' //HBD debug log start char „H‟
#define IOP_START_HBD2 'B'
#define IOP_START_HBD3 'D'
#define NMEA_ID_QUE_SIZE 0x0100
#define NMEA_RX_QUE_SIZE 0x8000
typedef enum
{
RXS_DAT_HBD, // receive HBD data
RXS_PRM_HBD2, // receive HBD preamble 2
RXS_PRM_HBD3, // receive HBD preamble 3
RXS_DAT, // receive NMEA data
RXS_DAT_DBG, // receive DBG data
RXS_ETX, // End-of-packet
} RX_SYNC_STATE_T;
struct
{
short inst_id; // 1 - NMEA, 2 - DBG, 3 - HBD
short dat_idx;
short dat_siz;
} id_que[NMEA_ID_QUE_SIZE]; 

char rx_que[NMEA_RX_QUE_SIZE];
unsigned short id_que_head;
unsigned short id_que_tail;
unsigned short rx_que_head;
RX_SYNC_STATE_T rx_state;
unsigned int u4SyncPkt;
unsigned int u4OverflowPkt;
unsigned int u4PktInQueue;
//Queue Functions
BOOL iop_init_pcrx( void )
{
/*----------------------------------------------------------
variables
----------------------------------------------------------*/
short i;
/*----------------------------------------------------------
initialize queue indexes
----------------------------------------------------------*/
id_que_head = 0;
id_que_tail = 0;
rx_que_head = 0;
/*----------------------------------------------------------
initialize identification queue
----------------------------------------------------------*/
for( i=0; i< NMEA_ID_QUE_SIZE; i++)
{
id_que[i].inst_id = -1;
id_que[i].dat_idx = 0;
}
/*----------------------------------------------------------
initialize receive state
----------------------------------------------------------*/
rx_state = RXS_ETX;
/*----------------------------------------------------------
initialize statistic information
----------------------------------------------------------*/
u4SyncPkt = 0;
u4OverflowPkt = 0;
u4PktInQueue = 0;
return TRUE;
}
/*********************************************************************
* PROCEDURE NAME:
* iop_inst_avail - Get available NMEA sentence information 

*
* DESCRIPTION:
* inst_id - NMEA sentence type
* dat_idx - start data index in queue
* dat_siz - NMEA sentence size
*********************************************************************/
BOOL iop_inst_avail(short *inst_id, short *dat_idx,
short *dat_siz)
{
/*----------------------------------------------------------
variables
----------------------------------------------------------*/
BOOL inst_avail;
/*----------------------------------------------------------
if packet is available then return id and index
----------------------------------------------------------*/
if ( id_que_tail != id_que_head )
{
*inst_id = id_que[ id_que_tail ].inst_id;
*dat_idx = id_que[ id_que_tail ].dat_idx;
*dat_siz = id_que[ id_que_tail ].dat_siz;
id_que[ id_que_tail ].inst_id = -1;
id_que_tail = ++id_que_tail & (unsigned short)(NMEA_ID_QUE_SIZE - 1);
inst_avail = TRUE;
if (u4PktInQueue > 0)
{
u4PktInQueue--;
}
}
else
{
inst_avail = FALSE;
}
return ( inst_avail );
} /* iop_inst_avail() end */
/*********************************************************************
* PROCEDURE NAME:
* iop_get_inst - Get available NMEA sentence from queue
*
* DESCRIPTION:
* idx - start data index in queue
* size - NMEA sentence size
* data - data buffer used to save NMEA sentence
*********************************************************************/ 

void iop_get_inst(short idx, short size, void *data)
{
/*----------------------------------------------------------
variables
----------------------------------------------------------*/
short i;
unsigned char *ptr;
/*----------------------------------------------------------
copy data from the receive queue to the data buffer
----------------------------------------------------------*/
ptr = (unsigned char *)data;
for (i = 0; i < size; i++)
{
*ptr = rx_que[idx];
ptr++;
idx = ++idx & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
}
} /* iop_get_inst() end */
/*********************************************************************
* PROCEDURE NAME:
* iop_pcrx_nmea - Receive NMEA code
*
* DESCRIPTION:
* The procedure fetch the characters between/includes '$' and <CR>.
* That is, character <CR><LF> is skipped.
* And the maximum size of the sentence fetched by this procedure is 256
* $xxxxxx*AA
*
*********************************************************************/
void iop_pcrx_nmea( unsigned char data )
{
/*----------------------------------------------------------
determine the receive state
----------------------------------------------------------*/
if (data == IOP_LF_DATA){
return;
}
switch (rx_state)
{
case RXS_DAT:
switch (data)
{
case IOP_CR_DATA:
// Count total number of sync packets

u4SyncPkt += 1;
id_que_head = ++id_que_head & (unsigned short)(NMEA_ID_QUE_SIZE - 1);
if (id_que_tail == id_que_head)
{
// Count total number of overflow packets
u4OverflowPkt += 1;
id_que_tail = ++id_que_tail & (unsigned short)(NMEA_ID_QUE_SIZE - 1);
}
else
{
u4PktInQueue++;
}
rx_state = RXS_ETX;
/*----------------------------------------------------------
set RxEvent signaled
----------------------------------------------------------*/
SetEvent(hRxEvent);
break;
case IOP_START_NMEA:
{
// Restart NMEA sentence collection
rx_state = RXS_DAT;
id_que[id_que_head].inst_id = 1;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
break;
}
default:
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
// if NMEA sentence length > 256, stop NMEA sentence collection
if (id_que[id_que_head].dat_siz == MAX_NMEA_STN_LEN)
{
id_que[id_que_head].inst_id = -1;
rx_state = RXS_ETX;
}
break;
}
break;
case RXS_ETX:

if (data == IOP_START_NMEA)
{
rx_state = RXS_DAT;
id_que[id_que_head].inst_id = 1;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
}
break;
default:
rx_state = RXS_ETX;
break;
}
} /* iop_pcrx_nmea() end */
/*********************************************************************
* PROCEDURE NAME:
* void iop_pcrx_nmea_dbg_hbd_bytes(unsigned char aData[], int i4NumByte)
* - Receive NMEA and debug log code
*
* DESCRIPTION:
* The procedure fetch the characters between/includes '$' and <CR>.
* That is, character <CR><LF> is skipped.
* And the maximum size of the sentence fetched by this procedure is 256
* $xxxxxx*AA
*
*********************************************************************/
void iop_pcrx_nmea_dbg_hbd_bytes(unsigned char aData[], int i4NumByte)
{
int i;
unsigned char data;
for (i = 0; i < i4NumByte; i++)
{
data = aData[i];
if (data == IOP_LF_DATA){
continue;
}
/*----------------------------------------------------------
determine the receive state
----------------------------------------------------------*/
switch (rx_state)
{
case RXS_DAT: 

switch (data)
{
case IOP_CR_DATA:
// Count total number of sync packets
u4SyncPkt += 1;
id_que_head = ++id_que_head & (unsigned short)(NMEA_ID_QUE_SIZE - 1);
if (id_que_tail == id_que_head)
{
// Count total number of overflow packets
u4OverflowPkt += 1;
id_que_tail = ++id_que_tail & (unsigned short)(NMEA_ID_QUE_SIZE - 1);
}
else
{
u4PktInQueue++;
}
rx_state = RXS_ETX;
/*----------------------------------------------------------
set RxEvent signaled
----------------------------------------------------------*/
SetEvent(hRxEvent);
break;
case IOP_START_NMEA:
{
// Restart NMEA sentence collection
rx_state = RXS_DAT;
id_que[id_que_head].inst_id = 1;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
break;
}
case IOP_START_DBG:
{
// Restart DBG sentence collection
rx_state = RXS_DAT_DBG;
id_que[id_que_head].inst_id = 2;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = data;


rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
break;
}
default:
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
// if NMEA sentence length > 256, stop NMEA sentence collection
if (id_que[id_que_head].dat_siz == MAX_NMEA_STN_LEN)
{
id_que[id_que_head].inst_id = -1;
rx_state = RXS_ETX;
}
break;
}
break;
case RXS_DAT_DBG:
switch (data)
{
case IOP_CR_DATA:
// Count total number of sync packets
u4SyncPkt += 1;
id_que_head = ++id_que_head & (unsigned short)(NMEA_ID_QUE_SIZE -
1);
if (id_que_tail == id_que_head)
{
// Count total number of overflow packets
u4OverflowPkt += 1;
id_que_tail = ++id_que_tail & (unsigned short)(NMEA_ID_QUE_SIZE - 1);
}
else
{
u4PktInQueue++;
}
rx_state = RXS_ETX;
/*----------------------------------------------------------
set RxEvent signaled
----------------------------------------------------------*/
SetEvent(hRxEvent);
break;
case IOP_START_NMEA:
{ 

// Restart NMEA sentence collection
rx_state = RXS_DAT;
id_que[id_que_head].inst_id = 1;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE -
1);
id_que[id_que_head].dat_siz++;
break;
}
case IOP_START_DBG:
{
// Restart DBG sentence collection
rx_state = RXS_DAT_DBG;
id_que[id_que_head].inst_id = 2;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE -
1);
id_que[id_que_head].dat_siz++;
break;
}
default:
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE -
1);
id_que[id_que_head].dat_siz++;
// if NMEA sentence length > 256, stop NMEA sentence collection
if (id_que[id_que_head].dat_siz == MAX_NMEA_STN_LEN)
{
id_que[id_que_head].inst_id = -1;
rx_state = RXS_ETX;
}
break;
}
break;
case RXS_DAT_HBD:
switch (data)
{
case IOP_CR_DATA:
// Count total number of sync packets 

u4SyncPkt += 1;
id_que_head = ++id_que_head & (unsigned short)(NMEA_ID_QUE_SIZE -
1);
if (id_que_tail == id_que_head)
{
// count total number of overflow packets
u4OverflowPkt += 1;
id_que_tail = ++id_que_tail & (unsigned short)(NMEA_ID_QUE_SIZE - 1);
}
else
{
u4PktInQueue++;
}
rx_state = RXS_ETX;
/*----------------------------------------------------------
set RxEvent signaled
----------------------------------------------------------*/
SetEvent(hRxEvent);
break;
case IOP_START_NMEA:
{
// Restart NMEA sentence collection
rx_state = RXS_DAT;
id_que[id_que_head].inst_id = 1;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE -
1);
id_que[id_que_head].dat_siz++;
break;
}
case IOP_START_DBG:
{
// Restart DBG sentence collection
rx_state = RXS_DAT_DBG;
id_que[id_que_head].inst_id = 2;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE -
1);
id_que[id_que_head].dat_siz++; 

break;
}
default:
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE -
1);
id_que[id_que_head].dat_siz++;
// if NMEA sentence length > 256, stop NMEA sentence collection
if (id_que[id_que_head].dat_siz == MAX_NMEA_STN_LEN)
{
id_que[id_que_head].inst_id = -1;
rx_state = RXS_ETX;
}
break;
}
break;
case RXS_ETX:
if (data == IOP_START_NMEA)
{
rx_state = RXS_DAT;
id_que[id_que_head].inst_id = 1;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
}
else if (data == IOP_START_DBG)
{
rx_state = RXS_DAT_DBG;
id_que[id_que_head].inst_id = 2;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = data;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
}
else if (data == IOP_START_HBD1)
{
rx_state = RXS_PRM_HBD2;
}
break;
case RXS_PRM_HBD2: 

if (data == IOP_START_HBD2)
{
rx_state = RXS_PRM_HBD3;
}
else
{
rx_state = RXS_ETX;
}
break;
case RXS_PRM_HBD3:
if (data == IOP_START_HBD3)
{
rx_state = RXS_DAT_HBD;
// Start to collect the packet
id_que[id_que_head].inst_id = 3;
id_que[id_que_head].dat_idx = rx_que_head;
id_que[id_que_head].dat_siz = 0;
rx_que[rx_que_head] = IOP_START_HBD1;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
rx_que[rx_que_head] = IOP_START_HBD2;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
rx_que[rx_que_head] = IOP_START_HBD3;
rx_que_head = ++rx_que_head & (unsigned short)(NMEA_RX_QUE_SIZE - 1);
id_que[id_que_head].dat_siz++;
}
else
{
rx_state = RXS_ETX;
}
break;
default:
rx_state = RXS_ETX;
break;
}
}
} /* iop_pcrx_nmea_dbg_hbd_bytes() end */

