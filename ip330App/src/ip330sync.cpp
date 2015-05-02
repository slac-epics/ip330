#define _NO_IP330_STATIC_
#include"drvIP330Private.h"
#include"bldPvClient.h"
#include<stdio.h>
#include<string.h>
#include<math.h>

#define IP330_THREAD_PRIORITY (epicsThreadPriorityMedium)
#define IP330_THREAD_STACK	(0x20000)

epicsMutexId    ip330SyncObject::master_lock = NULL;

static int ThreadStart(void *param)
{
    ip330SyncObject *sobj = new ip330SyncObject((IP330_CARD *)param);
    sobj->Wait();         /* Wait for a go! */
    return sobj->poll();  /* This never returns. */
}

extern "C" void ip330ThreadStart(IP330_CARD *ip330)
{
    ip330SyncObject::Init();
    ip330->sync_thread = epicsThreadMustCreate(ip330->cardname, IP330_THREAD_PRIORITY,
                                               IP330_THREAD_STACK,
                                               (EPICSTHREADFUNC)ThreadStart, (void *) ip330);
}

void ip330SyncObject::Init(void)
{
    if (!master_lock) {
        master_lock = epicsMutexMustCreate();
        epicsMutexLock(master_lock);
    }
}

void ip330SyncObject::Wait(void)
{
    epicsMutexLock(master_lock);
    epicsMutexUnlock(master_lock);
}

void ip330SyncObject::StartAll(void)
{
    epicsMutexUnlock(master_lock);
}

ip330SyncObject::ip330SyncObject(IP330_CARD *_ip330)
{
    ip330 = _ip330;
    SetParams(ip330->trig, ip330->gen, ip330->delay, ip330->sync);
}

DataObject *ip330SyncObject::Acquire(void)
{
    while (ip330->rd == ip330->wr) {
        epicsEventWait(ip330->data_avail);
    }
    return new DataObject(ip330->data[ip330->rd++ & N_DATABUF_MASK]);
}

int ip330SyncObject::CheckError(DataObject *dobj)
{
    return 0;
}

#ifdef	ENABLE_CONTEXT_TIMER_DIAGS
static ContextTimerMax ip330ISRLoopContextTimer( "ip330ISRLoop" );
static ContextTimerMax ip330ISRUniformContContextTimer( "ip330ISRUniformCont" );
#endif	//	ENABLE_CONTEXT_TIMER_DIAGS

void ip330SyncObject::QueueData(DataObject *dobj, epicsTimeStamp &evt_time)
{
    int loop, wr = ip330->sum_wr;
    volatile IP330_HW_MAP	*pHardware = ip330->pHardware;	/* controller registers */
    UINT16 *data = (UINT16 *)dobj->data;

    /* Load sum_data, if reach avg_times and cont. mode needs reset, stop scan */
    for(loop = ip330->start_channel; loop <= ip330->end_channel; loop++) {
        UINT32 tmp;
        UINT32 tmp_avgtimes, tmp_sum;

        tmp = ip330->sum_data[wr][loop];
        tmp_sum = (tmp & 0x00FFFFFF);
        tmp_avgtimes = ((tmp & 0xFF000000) >> 24) + 1;

        if( tmp == 0xFFFFFFFF || tmp_avgtimes >= ip330->avg_times) {
            /* No data or already reached the average times */
            tmp_sum = 0;
            tmp_avgtimes = 0;
        }

        if((data[MAX_IP330_CHANNELS] & 1) &&  (tmp_avgtimes+1) <= ip330->avg_times) {
            tmp_sum += data[loop];
            tmp_avgtimes += 1;
        }
        if((data[MAX_IP330_CHANNELS] & 2) &&  (tmp_avgtimes+1) <= ip330->avg_times) {
            tmp_sum += data[loop+16];
            tmp_avgtimes += 1;
        }
        ip330->sum_data[wr][loop] = ( (tmp_sum & 0x00FFFFFF) | ((tmp_avgtimes - 1) << 24) );

        if ( tmp_avgtimes >= ip330->avg_times && loop == ip330->end_channel ) {
            if ((ip330->avg_rst != 0) && (ip330->scan_mode == SCAN_MODE_UNIFORMCONT)) {
#ifdef	ENABLE_CONTEXT_TIMER_DIAGS
                ContextTimer	contextTimer( ip330ISRUniformContContextTimer );
#endif	//	ENABLE_CONTEXT_TIMER_DIAGS
                /* Save the current control register settings */
                UINT16		saved_ctrl = pHardware->controlReg;
                pHardware->controlReg = (saved_ctrl) & 0xF8FF; /* Disable scan */
                while( pHardware->controlReg & 0x0700 ) {	/* How long are we stuck in this loop? */
#ifdef	ENABLE_CONTEXT_TIMER_DIAGS
                    ContextTimer	contextTimer( ip330ISRLoopContextTimer );
#endif	//	ENABLE_CONTEXT_TIMER_DIAGS
                }

                /* Work around hardware bug of uniform continuous mode */
                pHardware->startChannel = ip330->start_channel;
                pHardware->controlReg	= saved_ctrl;
                pHardware->startConvert	= 1;
            }
            /* OK, make this available for the reader! */
            epicsMutexLock(ip330->lock);
            ip330->sum_time = evt_time;
            if (ip330->bldID)
                SendBld(wr);
            ip330->sum_wr = 1 - wr;
            epicsMutexUnlock(ip330->lock);
            scanIoRequest(ip330->ioscan);
        }
    }
}

void ip330SyncObject::SendBld(int wr)
{
    char buf[16*sizeof(double) + sizeof(int)];
    int off = 0;
    int loop, i, cnt;
    double v;

    cnt = ip330->end_channel + 1 - ip330->start_channel;
    memcpy(&buf[off], &cnt, sizeof(int));
    off += sizeof(int);
    for(i = 0, loop = ip330->start_channel; loop <= ip330->end_channel; i++, loop++) {
        UINT32	tmp = ip330->sum_data[wr][loop];

        if(tmp == 0xFFFFFFFF) {
            v = NAN;
            memcpy(&buf[off], &v, sizeof(double));
            off += sizeof(double);
            continue;
        }

        double	tmp_sum		 = tmp & 0x00FFFFFF;
	double	tmp_avgtimes = ( (tmp & 0xFF000000) >> 24 ) + 1;
	double	value		 = ip330->adj_slope[ip330->gain[loop]]
                                    * ( tmp_sum/tmp_avgtimes + ip330->adj_offset[ip330->gain[loop]] );
        int pvalue = static_cast<int>( value );
        v = pvalue * ip330->eslo + ip330->eoff;
        memcpy(&buf[off], &v, sizeof(double));
        off += sizeof(double);
    }
    BldSendPacket(ip330->bldClient, ip330->bldID, 0x10000 | 94, &ip330->sum_time, &buf,
                  sizeof(unsigned int) + sizeof(double) * cnt);
}

void ip330SyncObject::DebugPrint(DataObject *dobj)
{
}
