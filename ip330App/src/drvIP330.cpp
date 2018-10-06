/****************************************************************/
/* This file implements driver support for IP330 ADC            */
/* Author: Sheng Peng, pengs@slac.stanford.edu, 650-926-3847    */
/****************************************************************/
#ifdef vxWorks
#include "logLib.h"
#endif

#include "drvIP330Lib.h"
#include "drvIP330Private.h"
#include "devLib.h"
#include "errlog.h"
#include "epicsThread.h"
#include "iocsh.h"
#include "ContextTimer.h"
#include <epicsExport.h>

extern "C" {

int     IP330_DRV_DEBUG = 0;

static IP330_CARD_LIST	ip330_card_list;
static int		card_list_inited=0;

/*****************************************************************/
/* Find IP330_CARD which matches the cardname from link list     */
/*****************************************************************/
IP330_ID ip330GetByName(char * cardname)
{
    IP330_ID pcard = NULL;

    if(!card_list_inited)  
		return NULL;

	for (	pcard = (IP330_ID)ellFirst((ELLLIST *)&ip330_card_list);
			pcard != NULL;
			pcard = (IP330_ID)ellNext((ELLNODE *)pcard) )
    {
        if ( 0 == strcmp(cardname, pcard->cardname) )
			break;
    }

    return pcard;
}

/*****************************************************************/
/* Find IP330_CARD which matches the carrier/slot from link list */
/*****************************************************************/
IP330_ID ip330GetByLocation(UINT16 carrier, UINT16 slot)
{
    IP330_ID pcard = NULL;

    if(!card_list_inited)  
		return NULL;

	for (	pcard = (IP330_ID)ellFirst((ELLLIST *)&ip330_card_list);
			pcard != NULL;
			pcard = (IP330_ID)ellNext((ELLNODE *)pcard) )
    {
        if ( (carrier == pcard->carrier) && (slot == pcard->slot) )  
			break;
    }

    return pcard;
}

/**************************************************************************************************************************************************************/
/*  Routine: ip330Create                                                                                                                                      */
/*                                                                                                                                                            */
/*  Purpose: Register a new IP-330 ADC module                                                                                                                 */
/*                                                                                                                                                            */
/*  Description:                                                                                                                                              */
/*                                                                                                                                                            */
/*  Checks all parameters, then creates a new card structure, initializes it and adds it to the end of the linked list.                                       */
/*                                                                                                                                                            */
/*  SYNOPSIS: int ip330Create(                                                                                                                                */
/*   		    char *cardname,	  Unique Identifier "ip330-1"                                                                                         */
/*    		    UINT16 carrier,	  Ipac Driver carrier card number                                                                                     */
/*   		    UINT16 slot,	  Slot number on IP carrier                                                                                           */
/*                  char *adcrange        This must be "-5to5D","-10to10D","0to5D","0to10D","-5to5S","-10to10S","0to5S","0to10S"                              */
/*                  char *channels,       This must be "ch1-ch12" or "ch2-ch30", available range is 0 to 15/31 depends on Differential or Single End          */
/*                  UINT32 gainL,         The lower two bits is gain for channel0, then channel 1 ... 15                                                      */
/*                  UINT32 gainH,         The lower two bits is gain for channel16, then channel 17 ... 31                                                    */
/*                  char *scanmode        Scan mode must be "mode-trgdir-AvgxR"                                                                               */
/*                                        mode could be "uniformCont", "uniformSingle", "burstCont", "burstSingle", "cvtOnExt"                                */
/*                                        trgdir could be "Input", "Output" except for "cvtOnExt" which must come with "Input"                                */
/*                                        AvgxN could be Avg10 means average 10 times, R means reset after average and only applicable to Continuous mode     */
/*                  char *timer,          "x*y@8MHz", x must be [64,255], y must be [1,65535]                                                                 */
/*                  UINT8  vector, char *triggerPV, char *delayPV, char *syncPV, int bldID)                                                                   */
/*  Example:                                                                                                                                                  */
/*            ip330Create("ip330_1", 0, 0, "0to5D", "ch1-ch10", 0x0, 0x0, "burstCont-Input-Avg10R", "64*2@8MHz", 0x66)                                        */
/**************************************************************************************************************************************************************/

static void ip330ISR(void * arg);

int ip330Create (char *cardname, UINT16 carrier, UINT16 slot, char *adcrange, char * channels, UINT32 gainL, UINT32 gainH, char *scanmode, char * timer, UINT8 vector, char *trigger, char *delay, char *sync, int bldID)
{
    DBADDR addr;
    epicsUInt32 *trig, *gen;
    static double zero = 0.0;
    double *dval = &zero;
    int status, loop;
    int start_channel, end_channel;
    char tmp_smode[64], tmp_trgdir[64], tmp_avg[64];
    int avg_times;
    int timer_prescaler, conversion_timer;

    if (trigger && !dbNameToAddr(trigger, &addr)) {
        trig = (epicsUInt32 *) addr.pfield;
        gen = trig + MAX_EV_TRIGGERS;
    } else {
        if (trigger) {
            printf("\n\n\nNo PV trigger named %s!\n\n\n", trigger);
            fflush(stdout);
        }
        trig = NULL;
        gen = NULL;
    }
    if (delay && !dbNameToAddr(delay, &addr)) {
        dval = (double *) addr.pfield;
    }

    /* calloc will put everything zero */
    IP330_ID	pcard	= reinterpret_cast<IP330_ID>( callocMustSucceed(1, sizeof(struct IP330_CARD), "ip330Create") );

    if(!card_list_inited)
    {/* Initialize the IP330 link list */
        ellInit( (ELLLIST *) &ip330_card_list);
        card_list_inited = 1;
    }

    pcard->lock = epicsMutexMustCreate();

    pcard->trig = trig;
    pcard->gen = gen;
    pcard->delay = dval;
    pcard->sync = sync ? strdup(sync) : strdup("");
    pcard->bldID = bldID;
    if (bldID != -1) {
        static int bldClient = 0;
        pcard->bldClient = bldClient++;
    }
    pcard->wr = 0;
    pcard->rd = 0;

    pcard->data_avail = epicsEventMustCreate(epicsEventEmpty);

    for(loop = 0; loop < N_GAINS; loop++)
    {
        pcard->adj_slope[loop] = 1.0;
        pcard->adj_offset[loop] = 0.0;
    }

    for(loop = 0; loop < MAX_IP330_CHANNELS; loop++)
    {
        pcard->sum_data[0][loop] = 0xFFFFFFFF;	/* Mark data is not available */
        pcard->sum_data[1][loop] = 0xFFFFFFFF;
    }
    pcard->sum_wr = 0;

    scanIoInit( &(pcard->ioscan) );

    /************************************ Parameters check ************************************/

    /* Check cardname */
    if( (!cardname) || (0 == strlen(cardname)) )
    {
        errlogPrintf ("ip330Create: No cardname specified!\n");
        status = -1;
        goto FAIL;
    }
    if( ip330GetByName(cardname) )
    {
        errlogPrintf ("ip330Create: %s already existed!\n", cardname);
        status = -1;
        goto FAIL;
    }
    pcard->cardname = epicsStrDup(cardname);

    /* Check if we really have the Acromag IP330 installed on carrier/slot */
    status = ipmValidate(carrier, slot, IP_MANUFACTURER_ACROMAG, IP_MODEL_ADC_330);
    if (status)
    {
        errlogPrintf ("ip330Create: Carrier %d Slot %d has no IP330\n", carrier, slot);
        goto FAIL;
    }
    if( ip330GetByLocation(carrier, slot) )
    {
        errlogPrintf ("ip330Create: IP330 on carrier %d slot %d is already registered!\n", carrier, slot);
        status = -1;
        goto FAIL;
    }
    /* Slot contains a real IP-330 module */
    pcard->carrier = carrier;
    pcard->slot = slot;

    /* Check ADC range and Input type (Differential of Single End) */
    for(loop=0; loop<N_RANGES*N_INPTYPS; loop++)
    {
        if(!strcmp(adcrange, rangeName[loop]))
			break;
    }
    if(loop < N_RANGES*N_INPTYPS)
    {
        pcard->inp_range = loop%N_RANGES;
        pcard->inp_typ = loop/N_RANGES;
		pcard->num_chnl =	(	pcard->inp_typ == INP_TYP_DIFF
							?	MAX_IP330_CHANNELS / 2
							:	MAX_IP330_CHANNELS              );

    }
    else
    {
        errlogPrintf ("ip330Create: adcrange %s is illegal for device %s\n", adcrange, cardname);
        status = -1;
        goto FAIL;
    }
    switch (pcard->inp_range) {
    case ADC_RANGE_B5V:
        pcard->eslo = 10.0 / (double) 0x10000;
        pcard->eoff = -5.0;
        break;
    case ADC_RANGE_B10V:
        pcard->eslo = 20.0 / (double) 0x10000;
        pcard->eoff = -10.0;
        break;
    case ADC_RANGE_U5V:
        pcard->eslo = 5.0 / (double) 0x10000;
        pcard->eoff = 0.0;
        break;
    case ADC_RANGE_U10V:
        pcard->eslo = 10.0 / (double) 0x10000;
        pcard->eoff = 0.0;
        break;
    }

    /* Check scan channel range */
    if(2 != sscanf(channels, channelsFormat, &start_channel, &end_channel) )
    {
        errlogPrintf ("ip330Create: channel range %s is illegal for device %s\n", channels, cardname);
        status = -1;
        goto FAIL;
    }
    if( (start_channel < 0) || (start_channel >= (int) pcard->num_chnl) ||
        (end_channel   < 0) || (end_channel   >= (int) pcard->num_chnl) ||
		(start_channel > end_channel) )
    {
        errlogPrintf ("ip330Create: channel range %s is illegal for device %s\n", channels, cardname);
        status = -1;
        goto FAIL;
    }
    pcard->start_channel = start_channel;
    pcard->end_channel = end_channel;
    pcard->chnl_mask = 0xFFFFFFFF >> (pcard->start_channel);
    pcard->chnl_mask <<= (32 - (pcard->end_channel - pcard->start_channel + 1));
    pcard->chnl_mask >>= (32 - pcard->end_channel - 1);

    /* Get gain for each channel */
    for(loop=0; loop<(MAX_IP330_CHANNELS/2); loop++)
    {
        pcard->gain[loop] = (gainL>>(loop*2))&0x3;
        pcard->gain[(MAX_IP330_CHANNELS/2)+loop] = (gainH>>(loop*2))&0x3; 
    }

    /* Check combined scan mode */
    if( 3 != sscanf(scanmode, "%[^-]-%[^-]-%[^-]", tmp_smode, tmp_trgdir, tmp_avg) )
    {
        errlogPrintf ("ip330Create: scan mode %s is illegal for device %s\n", scanmode, cardname);
        status = -1;
        goto FAIL;
    }

    /* Check Scan Mode */
    for(loop=0; loop<N_SCANMODES; loop++)
    {
        if(!strcmp(tmp_smode, scanModeName[loop])) break;
    }
    if(loop < N_SCANMODES)
    {
        pcard->scan_mode = loop;
    }
    else
    {
        errlogPrintf ("ip330Create: scan mode %s is illegal for device %s\n", scanmode, cardname);
        status = -1;
        goto FAIL;
    }

    /* Check Trigger Direction */
    for(loop=0; loop<N_TRGDIRS; loop++)
    {
        if(!strcmp(tmp_trgdir, trgDirName[loop])) break;
    }
    if(loop < N_TRGDIRS)
    {
        pcard->trg_dir = loop;
    }
    else
    {
        errlogPrintf ("ip330Create: scan mode %s is illegal for device %s\n", scanmode, cardname);
        status = -1;
        goto FAIL;
    }

    if( 1 != sscanf(tmp_avg, avgFormat, &avg_times) )
    {
        errlogPrintf ("ip330Create: scan mode %s is illegal for device %s\n", scanmode, cardname);
        status = -1;
        goto FAIL;
    }
    if(avg_times >= 1 && avg_times <= MAX_AVG_TIMES)
    {
        pcard->avg_times = avg_times;
    }
    else
    {
        errlogPrintf ("ip330Create: scan mode %s is illegal for device %s\n", scanmode, cardname);
        status = -1;
        goto FAIL;
    }
    if(strchr(tmp_avg, 'R'))
        pcard->avg_rst = 1;
    else
        pcard->avg_rst = 0;

    /* Check timer */
    if( 2 != sscanf(timer, timerFormat, &timer_prescaler, &conversion_timer) )
    {
        errlogPrintf ("ip330Create: timer %s is illegal for device %s\n", timer, cardname);
        status = -1;
        goto FAIL;
    }
    if( (timer_prescaler >= MIN_TIMER_PRESCALER && timer_prescaler <= MAX_TIMER_PRESCALER) &&
        (conversion_timer >= MIN_CONVERSION_TIMER && conversion_timer <= MAX_CONVERSION_TIMER) )
    {
        pcard->timer_prescaler = timer_prescaler;
        pcard->conversion_timer = conversion_timer;
    }
    else
    {
        errlogPrintf ("ip330Create: timer %s is illegal for device %s\n", timer, cardname);
        status = -1;
        goto FAIL;
    }

    /* Interrupt Vector */
    pcard->vector = vector;

    /* Hardware pointer */
    pcard->pHardware   = (IP330_HW_MAP *) ipmBaseAddr(carrier, slot, ipac_addrIO);

    /* Install ISR */
    if ( ipmIntConnect( carrier, slot, vector, ip330ISR, pcard ) )
    {
        errlogPrintf ("ip330Create: intConnect failed for device %s\n", cardname);
        status = -1;
        goto FAIL;
    }

    /* We successfully allocate all resource */
    ellAdd( (ELLLIST *)&ip330_card_list, (ELLNODE *)pcard);

    pcard->pHardware->controlReg = 0x0;
    ipmIrqCmd(carrier, slot, 0, ipac_irqEnable);

    ip330Calibrate(pcard);
    ip330Configure(pcard);
    ip330ThreadStart(pcard);

    return 0;

FAIL:
    if(pcard->lock) epicsMutexDestroy(pcard->lock);
    if(pcard->cardname) free(pcard->cardname);
    free(pcard);
    return status;
}

/*****************************************************************************************************************/
/* Since there is only one ADC and one Programmable Gain, we don't have to calibrate each channel.               */
/* We just calibrate ADC with different gain under certain input range.                                          */
/* Since calibration will interrupt normal scanning, so usually calibration should only happen in initialization.*/
/* Since calibration will clear the control register setting, make sure to run ip330Configure after calibration. */
/* This function can be only called from task level.                                                             */
/*****************************************************************************************************************/
void ip330Calibrate(IP330_ID pcard)
{
    int loopgain, loopchnl;
    int ntimes=0;

    long sum;
    double count_callo;
    double count_calhi;

    double m;

    if(!pcard)
    {
        errlogPrintf("ip330Calibrate called with NULL pointer!\n");
        return;
    }

    for(loopgain=0; loopgain < N_GAINS; loopgain++)
    {
        /* Disable Scan and Interrupt */ 
        pcard->pHardware->controlReg = 0x0;
        epicsThreadSleep(0.1); /* If an interrupt already generated before we disable interrupt, this delay will allow the ISR done. */

        pcard->pHardware->startChannel = 0;
        pcard->pHardware->endChannel = MAX_IP330_CHANNELS-1;

        for (loopchnl = 0; loopchnl < MAX_IP330_CHANNELS; loopchnl++) 
        {
            pcard->pHardware->gain[loopchnl] = loopgain;
        }

        /* determine count_callo */
        pcard->pHardware->controlReg = calSettings[pcard->inp_range][loopgain].ctl_callo;
        epicsThreadSleep(0.1); /* A delay of 5us needed according to manual */
        pcard->pHardware->startConvert = 0x0001;
        ntimes = 0;
        while(ntimes++ < 1000)
        {
            if( (pcard->pHardware->newData[0]==0xffff) && (pcard->pHardware->newData[1]==0xffff) ) break;
        }
        if(ntimes >= 1000)
        {
            errlogPrintf("Somehow the data for %s is not ready on time for calibration!\n", pcard->cardname);
            /* Disable Scan and Interrupt */ 
            pcard->pHardware->controlReg = 0x0;
            return;
        }
        sum = 0;
        for (loopchnl = 0; loopchnl < MAX_IP330_CHANNELS; loopchnl++)
            sum += pcard->pHardware->data[loopchnl];

        count_callo = ((double)sum)/(double)MAX_IP330_CHANNELS;

        /* determine count_calhi */
        pcard->pHardware->controlReg = calSettings[pcard->inp_range][loopgain].ctl_calhi;
        epicsThreadSleep(0.1); /* A delay of 5us needed according to manual */
        pcard->pHardware->startConvert = 0x0001;
        ntimes = 0;
        while(ntimes++ < 1000)
        {
            if( (pcard->pHardware->newData[0]==0xffff) && (pcard->pHardware->newData[1]==0xffff) ) break;
        }
        if(ntimes >= 1000)
        {
            errlogPrintf("Somehow the data for %s is not ready on time for calibration!\n", pcard->cardname);
            /* Disable Scan and Interrupt */ 
            pcard->pHardware->controlReg = 0x0;
            return;
        }
        sum = 0;
        for (loopchnl = 0; loopchnl < MAX_IP330_CHANNELS; loopchnl++)
            sum += pcard->pHardware->data[loopchnl];

        count_calhi = ((double)sum)/(double)MAX_IP330_CHANNELS;

        /* Calculate slope and offset according manual */
        m = pgaGain[loopgain] * (calSettings[pcard->inp_range][loopgain].volt_calhi - calSettings[pcard->inp_range][loopgain].volt_callo) / (count_calhi - count_callo);
        epicsMutexLock(pcard->lock);
        pcard->adj_slope[loopgain] = (65536.0 * m) / calSettings[pcard->inp_range][loopgain].ideal_span;
        pcard->adj_offset[loopgain] = ( (calSettings[pcard->inp_range][loopgain].volt_callo * pgaGain[loopgain]) - calSettings[pcard->inp_range][loopgain].ideal_zero )/m - count_callo;
        epicsMutexUnlock(pcard->lock);

        if(IP330_DRV_DEBUG)
            printf("IP330 %s: realdata = %g * (rawdata + %g)\n", pcard->cardname, pcard->adj_slope[loopgain], pcard->adj_offset[loopgain]);
    }

    /* Disable Scan and Interrupt */ 
    pcard->pHardware->controlReg = 0x0;
    return;
}

static inline int readData( volatile IP330_HW_MAP *	pHardware, int index )
{
	return pHardware->data[index];
}

/****************************************************************/
/* We don't allow change configuration during running so far.   */
/* If we do, the nornal procedure to call ip330Configure is:    */
/* 1. Get mutex semaphore                                       */
/* 2. Disable IP330 interrupt                                   */
/* 3. Delay a while to make sure no pending IRQ from this IP330 */
/* 4. Change whatever in IP330_CARD                             */
/* 5. Call ip330Configure                                       */
/* 6. Release mutex semaphore                                   */
/* This function cab be only called from task level             */
/* Since these procedure takes time, so the mutex semaphore here*/
/* must NOT be the one we are using in synchronous data read.   */
/* If data read wants to mutex with re-configuration, take this */
/* mutex semaphore without wait.                                */
/****************************************************************/
void ip330Configure(IP330_ID pcard)
{
    UINT16 tmp_ctrl;
    int loop;

    if(!pcard)
    {
        errlogPrintf("ip330Configure called with NULL pointer!\n");
        return;
    }

    /*-------------------------------------------------------------*/
    /* Initiallize the ADC control register.                       */
    /*-------------------------------------------------------------*/

    /* Stop scan and interrupt */
    pcard->pHardware->controlReg = 0x0000;

    /* Clear newdata and misseddata register and data */
    for (loop = 0; loop < MAX_IP330_CHANNELS; loop++)
    {
		(void) readData( pcard->pHardware, loop );
        pcard->sum_data[0][loop] = 0xFFFFFFFF;	/* Mark data is not available */
        pcard->sum_data[1][loop] = 0xFFFFFFFF;
    }

    tmp_ctrl	=	CTRL_REG_STRGHT_BINARY
				|	( pcard->trg_dir	<< CTRL_REG_TRGDIR_SHFT   )
				|	( pcard->inp_typ	<< CTRL_REG_INPTYP_SHFT   )
				|	( pcard->scan_mode	<< CTRL_REG_SCANMODE_SHFT )
				|	CTRL_REG_INTR_CTRL;

    if(pcard->scan_mode != SCAN_MODE_CVTONEXT && pcard->scan_mode != SCAN_MODE_BURSTSINGLE)
        tmp_ctrl |= CTRL_REG_TIMR_ENBL;

    /* timer prescaler register */
    pcard->pHardware->timerPrescaler = pcard->timer_prescaler;

    /* interrupt vector register */
    pcard->pHardware->intVector = pcard->vector;

    /* conversion timer count register */
    pcard->pHardware->conversionTimer = pcard->conversion_timer;

    /* end start channel register */
    pcard->pHardware->endChannel = pcard->end_channel;
    pcard->pHardware->startChannel = pcard->start_channel;

    /* set gain */
    for (loop = 0; loop < MAX_IP330_CHANNELS; loop++)
        pcard->pHardware->gain[loop] = pcard->gain[loop];

    pcard->pHardware->controlReg = tmp_ctrl;

    /* Delay at least 5us */
    epicsThreadSleep(0.01);

    /* Make sure startConvert is set when Convert on Ext Trigger Only mode */
    if( pcard->scan_mode == SCAN_MODE_CVTONEXT )
		pcard->pHardware->startConvert = 0x1;

    return;
}

/****************************************************************/
/* Read data for particular channel, do average and correction   */
/****************************************************************/
int ip330Read(IP330_ID pcard, UINT16 channel, signed int * pvalue, epicsTimeStamp *ts)
{
    if(!pcard)
    {
        errlogPrintf("ip330Read called with NULL pointer!\n");
        return -1;
    }

    epicsMutexLock(pcard->lock);

    if(channel < pcard->start_channel || channel > pcard->end_channel)
    {
        errlogPrintf("Bad channel number %d in ip330Read card %s\n", channel, pcard->cardname);
        epicsMutexUnlock(pcard->lock);
        return -1;
    }

    /* The raw value we read from hardware is UINT16 */
    /* But after calibration, it might be a little bit wider range */
    UINT32	tmp = pcard->sum_data[1 - pcard->sum_wr][channel];

    if(tmp == 0xFFFFFFFF)
    {
        //MCB        errlogPrintf("No data for channel number %d in ip330Read card %s\n", channel, pcard->cardname);
        epicsMutexUnlock(pcard->lock);
        return -1;
    }

    double	tmp_sum		 = tmp & 0x00FFFFFF;
	double	tmp_avgtimes = ( (tmp & 0xFF000000) >> 24 ) + 1;
	double	value		 =	pcard->adj_slope[pcard->gain[channel]]
						*	( tmp_sum/tmp_avgtimes + pcard->adj_offset[pcard->gain[channel]] );
    *pvalue = static_cast<int>( value );
    *ts = pcard->sum_time;

    epicsMutexUnlock(pcard->lock);

    return 0;
}


/****************************************************************/
/* Read data for paticular channel, do average and correction   */
/****************************************************************/
IOSCANPVT * ip330GetIoScanPVT(IP330_ID pcard)
{
    if(!pcard)
    {
        errlogPrintf("ip330GetIoScanPVT called with NULL pointer!\n");
        return NULL;
    }
    return &(pcard->ioscan);
}

/***********************************************************************/
/* Read data from mailbox,  check dual level buffer if needed          */
/* Put data into sum_data, stop scan and trigger record scan if needed */
/***********************************************************************/
#ifdef	ENABLE_CONTEXT_TIMER_DIAGS
static ContextTimerMax ip330ISRContextTimer( "ip330ISR" );
#endif	//	ENABLE_CONTEXT_TIMER_DIAGS
void ip330ISR(void * arg)
{
#ifdef	ENABLE_CONTEXT_TIMER_DIAGS
    ContextTimer	contextTimer( ip330ISRContextTimer );
#endif	//	ENABLE_CONTEXT_TIMER_DIAGS
    int loop;
    UINT32 newdata_flag, misseddata_flag;
    UINT32 buf0_avail=0, buf1_avail=0;
    char buf[32];

    IP330_ID					pcard	= (IP330_ID)arg;
    UINT16 *data = pcard->data[pcard->wr & N_DATABUF_MASK];
    volatile IP330_HW_MAP	*	pHardware = pcard->pHardware;	/* controller registers */

    /* disable ints on this module first */
    ipmIrqCmd(pcard->carrier, pcard->slot, 0, ipac_irqDisable);

    if ( IP330_DRV_DEBUG > 10 )
		epicsInterruptContextMessage("IP330 ISR called\n");

    if(pcard->inp_typ == INP_TYP_DIFF)
    {	/* Input type is differential, we have buf0 and buf1 */
        UINT16 newdata_flag0, newdata_flag1;
        UINT16 misseddata_flag0, misseddata_flag1;

        newdata_flag0 = (pHardware->newData[0]) & pcard->chnl_mask;
        newdata_flag1 = (pHardware->newData[1]) & pcard->chnl_mask;
        newdata_flag = ((newdata_flag1 << 16) | newdata_flag0);

        misseddata_flag0 = (pHardware->missedData[0]) & pcard->chnl_mask;
        misseddata_flag1 = (pHardware->missedData[1]) & pcard->chnl_mask;
        misseddata_flag = ((misseddata_flag1 << 16) | misseddata_flag0);
        
        if(newdata_flag0 == pcard->chnl_mask)
        {/* Read Data from buf0 */
            for(loop = pcard->start_channel; loop <= pcard->end_channel; loop++)
            {
                data[loop] = pHardware->data[loop];
            }
            buf0_avail = 1;
        }
        if(newdata_flag1 == pcard->chnl_mask)
        {/* Read Data from buf1 */
            for(loop = pcard->start_channel; loop <= pcard->end_channel; loop++)
            {
                data[loop+16] = pHardware->data[loop+16];
            }
            buf1_avail = 2;
        }
    }
    else
    {	/* Input type is single-end, there is only buf0 */
        newdata_flag = ((pHardware->newData[1]  << 16) | pHardware->newData[0]);
        newdata_flag &= pcard->chnl_mask;

        misseddata_flag = ((pHardware->missedData[1]  << 16) | pHardware->missedData[0]);
        misseddata_flag &= pcard->chnl_mask;

        if(newdata_flag == pcard->chnl_mask)
        {/* Read Data */
            for(loop = pcard->start_channel; loop <= pcard->end_channel; loop++)
            {
                data[loop] = pHardware->data[loop];
            }
            buf0_avail = 1;
        }
    }

    if(buf0_avail || buf1_avail)
    {
        data[MAX_IP330_CHANNELS] = buf0_avail | buf1_avail;
        pcard->wr++;   /* Do we want to check for wrap around, or is that just paranoia? */
        epicsEventSignal(pcard->data_avail);
    }
    else
    {
#ifdef vxWorks
        logMsg("Unexpected new data flag 0x%08x\n", newdata_flag, 0,0,0,0,0);
#elif defined(__rtems__)
        printf("Unexpected new data flag 0x%08x\n", newdata_flag);
#else
        epicsInterruptContextMessage("Unexpected new data flag\n");
#endif
    }

    if(misseddata_flag != 0)
    {
        sprintf(buf, "Missed data flag 0x%x mask 0x%x\n", misseddata_flag, pcard->chnl_mask);
        epicsInterruptContextMessage(buf);
    }

    ipmIrqCmd(pcard->carrier, pcard->slot, 0, ipac_irqClear);
    ipmIrqCmd(pcard->carrier, pcard->slot, 0, ipac_irqEnable);

	// Note: It's not good to clear scan mode or set startConvert in the ISR
	// Clearing the scan mode kills the ongoing scan forcing you
	// to set startConvert again, however, that causes an immediate scan
	// without waiting for a timeout resulting in roughly 15KHz interrupt rate.
        //  pHardware->controlReg = saved_ctrl;
	//  pHardware->startConvert = 1;
}

/***********************************************************************/
/* Software trigger                                                    */
/***********************************************************************/
void ip330StartConvert(IP330_ID pcard)
{
    if(pcard)
    {
        if(pcard->trg_dir == TRG_DIR_OUTPUT)
            pcard->pHardware->startConvert = 0x1;
    }
}

void ip330StartConvertByName(char * cardname)
{
    IP330_ID pcard = ip330GetByName(cardname);
    ip330StartConvert(pcard);
}
/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/

/* implementation */
static long ip330Report(int level)
{
    IP330_ID pcard;

    printf("\n"IP330_DRV_VERSION"\n\n");

    if(!card_list_inited)
    {
        printf("IP330 card link list is not inited yet!\n\n");
        return 0;
    }

    if(level > 1)   /* we only get into link list for detail when user wants */
    {
        for (	pcard = (IP330_ID)ellFirst((ELLLIST *)&ip330_card_list);
				pcard != NULL;
				pcard = (IP330_ID)ellNext((ELLNODE *)pcard) )
        {
            printf("\tIP330 card %s is installed on carrier %d slot %d\n", pcard->cardname, pcard->carrier, pcard->slot);

            if(level > 2)
            {
                printf("\tInput range is %s, chnl%d~chnl%d is in use, scan mode is %s\n", 
                         rangeName[pcard->inp_typ*N_RANGES+pcard->inp_range], pcard->start_channel, 
                         pcard->end_channel, scanModeName[pcard->scan_mode]);
                printf("\tTrigger direction is %s, average %d times %s reset, timer is %gus\n", 
                         trgDirName[pcard->trg_dir], pcard->avg_times, pcard->avg_rst?"with":"without", 
                         pcard->timer_prescaler*pcard->conversion_timer/8.0);
                printf("\tIO space is at %p\n", pcard->pHardware);
    			printf("\tControl Reg is 0x%X\n", pcard->pHardware->controlReg );
            }
        }
    }

    return 0;
}

const struct drvet drvIP330 = 
{
	2,				/* 2 Table Entries */
	(DRVSUPFUN) ip330Report,	/* Driver Report Routine */
	NULL				/* Driver Initialization Routine */
};

epicsExportAddress(drvet,drvIP330);

#ifndef	NO_EPICS
static const iocshArg ip330ReportArg0 = {"interest", iocshArgInt};
static const iocshArg * const ip330ReportArgs[1] = {&ip330ReportArg0};
static const iocshFuncDef ip330ReportFuncDef = {"ip330Report",1,ip330ReportArgs};
static void ip330ReportCallFunc(const iocshArgBuf *args)
{
    ip330Report(args[0].ival);
}

/* ip330StartConvByName( char *pName); */

static const iocshArg ip330StartConvertByNameArg0 = {"pName",iocshArgPersistentString};
static const iocshArg * const ip330StartConvertByNameArgs[1] = {
    &ip330StartConvertByNameArg0 };

static const iocshFuncDef ip330StartConvertByNameFuncDef =
    {"ip330StartConvertByName",1,ip330StartConvertByNameArgs};

static void ip330StartConvertByNameCallFunc(const iocshArgBuf *arg)
{
    ip330StartConvertByName(arg[0].sval);
}

/* ip330Create( char *pName, unsigned short card, unsigned short slot, char *modeNamer); */

static const iocshArg ip330CreateArg0 = {"pName",iocshArgPersistentString};
static const iocshArg ip330CreateArg1 = {"card", iocshArgInt};
static const iocshArg ip330CreateArg2 = {"slot", iocshArgInt};
static const iocshArg ip330CreateArg3 = {"a2drange",iocshArgString};
static const iocshArg ip330CreateArg4 = {"channels",iocshArgString};
static const iocshArg ip330CreateArg5 = {"gainL",iocshArgInt};
static const iocshArg ip330CreateArg6 = {"gainH",iocshArgInt};
static const iocshArg ip330CreateArg7 = {"scanmode",iocshArgString};
static const iocshArg ip330CreateArg8 = {"timer",iocshArgString};
static const iocshArg ip330CreateArg9 = {"intvec",iocshArgInt};
static const iocshArg ip330CreateArg10= { "triggerPV",	iocshArgString };
static const iocshArg ip330CreateArg11= { "delayPV",		iocshArgString };
static const iocshArg ip330CreateArg12= { "syncPV",		iocshArgString };
static const iocshArg ip330CreateArg13= { "bldID",		iocshArgInt };

static const iocshArg * const ip330CreateArgs[14] = {
    &ip330CreateArg0, &ip330CreateArg1, &ip330CreateArg2, &ip330CreateArg3, &ip330CreateArg4,
    &ip330CreateArg5, &ip330CreateArg6, &ip330CreateArg7, &ip330CreateArg8, &ip330CreateArg9,
    &ip330CreateArg10, &ip330CreateArg11, &ip330CreateArg12, &ip330CreateArg13};

static const iocshFuncDef ip330CreateFuncDef =
    {"ip330Create",14,ip330CreateArgs};

static void ip330CreateCallFunc(const iocshArgBuf *arg)
{
    ip330Create(arg[0].sval, arg[1].ival, arg[2].ival, arg[3].sval, arg[4].sval,
                arg[5].ival, arg[6].ival, arg[7].sval, arg[8].sval, arg[9].ival,
                arg[10].sval, arg[11].sval, arg[12].sval, arg[13].ival);
}

static const iocshArg ip330StartArg0 = { "level",		iocshArgInt };

static const iocshArg * const ip330StartArgs[1] = {
    &ip330StartArg0};

static const iocshFuncDef ip330StartFuncDef =
    {"ip330Start",1,ip330StartArgs};

static void ip330StartCallFunc(const iocshArgBuf *arg)
{
    ip330SyncObject::StartAll();
}

// LOCAL void drvIP330Registrar(void) {
void drvIP330Registrar(void) {
    iocshRegister(&ip330ReportFuncDef,ip330ReportCallFunc); 
    iocshRegister(&ip330StartConvertByNameFuncDef,ip330StartConvertByNameCallFunc);
    iocshRegister(&ip330CreateFuncDef,ip330CreateCallFunc);
    iocshRegister(&ip330StartFuncDef,ip330StartCallFunc);
}
epicsExportRegistrar(drvIP330Registrar);
epicsExportAddress( int, IP330_DRV_DEBUG );
#endif /* NO_EPICS */

} // extern "C"
