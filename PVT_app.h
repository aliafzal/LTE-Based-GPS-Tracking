/*********************************************************************************************
   FILE:          PVT_app.h
----------------------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2016 STMicroelectronics
**********************************************************************************************/

#include <gpOS.h>

struct atcChannel;

extern void pvt_app_init(const struct atcChannel *atc);
//extern void pvt_app_init(gpOS_partition_t *part, const struct atcChannel *atc);

