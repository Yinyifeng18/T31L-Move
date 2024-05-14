/*
 * sample-Encoder-h264-IVS-move.c
 *
 * Copyright (C) 2016 Ingenic Semiconductor Co.,Ltd
 */

#include <string.h>
#include <imp/imp_log.h>
#include <imp/imp_common.h>
#include <imp/imp_system.h>
#include <imp/imp_framesource.h>
#include <imp/imp_encoder.h>
#include <imp/imp_ivs.h>
#include <imp/imp_ivs_base_move.h>
#include <limits.h>

#include "sample-common.h"

#define TAG "Sample-Encoder-h264-IVS-base-move"
#define SAD_MODE_SIZE 8  //sadMode为0时表示检测区域大小8*8，这个宏定义为8;
#define DEBUG_PRINT


extern struct chn_conf chn[];

#ifdef SUPPORT_RGB555LE
#include "bgramapinfo_rgb555le.h"
#else
#include "bgramapinfo.h"
#endif


#define OSD_LETTER_NUM 20

int grpNum = 0;
IMPRgnHandle *prHander;

IMPPoint  p00;  /**<左上角点坐标信息  */
IMPPoint  p01;  /**<左上角点坐标信息  */
 
#define ROWS 45  
#define COLS 80  
int array[ROWS][COLS] = {0};

unsigned char printRecflg = 0;
unsigned char DispCnt = 0;

static int sample_ivs_move_init(int grp_num)
{
	int ret = 0;

	ret = IMP_IVS_CreateGroup(grp_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_CreateGroup(%d) failed\n", grp_num);
		return -1;
	}
	return 0;
}

static int sample_ivs_move_exit(int grp_num)
{
	int ret = 0;

	ret = IMP_IVS_DestroyGroup(grp_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_DestroyGroup(%d) failed\n", grp_num);
		return -1;
	}
	return 0;
}

static int sample_ivs_move_start(int grp_num, int chn_num, IMPIVSInterface **interface)
{
	int ret = 0;
	IMP_IVS_BaseMoveParam param;

	memset(&param, 0, sizeof(IMP_IVS_BaseMoveParam));
	param.skipFrameCnt = 3;
	param.referenceNum = 4;
	param.sadMode = 0;
	param.sense = 3;
	param.frameInfo.width = SENSOR_WIDTH_SECOND;
	param.frameInfo.height = SENSOR_HEIGHT_SECOND;

	*interface = IMP_IVS_CreateBaseMoveInterface(&param);
	if (*interface == NULL) {
		IMP_LOG_ERR(TAG, "IMP_IVS_CreateGroup(%d) failed\n", grp_num);
		return -1;
	}

	ret = IMP_IVS_CreateChn(chn_num, *interface);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_CreateChn(%d) failed\n", chn_num);
		return -1;
	}

	ret = IMP_IVS_RegisterChn(grp_num, chn_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_RegisterChn(%d, %d) failed\n", grp_num, chn_num);
		return -1;
	}

	ret = IMP_IVS_StartRecvPic(chn_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_StartRecvPic(%d) failed\n", chn_num);
		return -1;
	}

	return 0;
}

static int sample_ivs_move_stop(int chn_num, IMPIVSInterface *interface)
{
	int ret = 0;

	ret = IMP_IVS_StopRecvPic(chn_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_StopRecvPic(%d) failed\n", chn_num);
		return -1;
	}
	sleep(1);

	ret = IMP_IVS_UnRegisterChn(chn_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_UnRegisterChn(%d) failed\n", chn_num);
		return -1;
	}

	ret = IMP_IVS_DestroyChn(chn_num);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_DestroyChn(%d) failed\n", chn_num);
		return -1;
	}

	IMP_IVS_DestroyBaseMoveInterface(interface);

	return 0;
}

#if 0
static int sample_ivs_set_sense(int chn_num, int sensor)
{
	int ret = 0;
	IMP_IVS_MoveParam param;
	int i = 0;

	ret = IMP_IVS_GetParam(chn_num, &param);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_GetParam(%d) failed\n", chn_num);
		return -1;
	}

	for( i = 0 ; i < param.roiRectCnt ; i++){
	  param.sense[i] = sensor;
	}

	ret = IMP_IVS_SetParam(chn_num, &param);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_IVS_SetParam(%d) failed\n", chn_num);
		return -1;
	}

	return 0;
}
#endif



  
void findMinMaxCoordinates(int array[ROWS][COLS], int *minRow, int *minCol, int *maxRow, int *maxCol) {  
    *minRow = INT_MAX;  
    *minCol = INT_MAX;  
    *maxRow = INT_MIN;  
    *maxCol = INT_MIN;  
  
    for (int i = 0; i < ROWS; ++i) {  
        for (int j = 0; j < COLS; ++j) {  
            if (array[i][j] == 1) {  
                if (i < *minRow) *minRow = i;  
                if (j < *minCol) *minCol = j;  
                if (i > *maxRow) *maxRow = i;  
                if (j > *maxCol) *maxCol = j;  
            }  
        }  
    }  
}  


static void *sample_ivs_move_get_result_process(void *arg)
{
	int i = 0, j = 0, ret = 0;
	int n,m;
	int flg = 0;
	int chn_num = (int)arg;
	int minRow, minCol, maxRow, maxCol;  
	IMP_IVS_BaseMoveOutput *result = NULL;

	for (i = 0; i < NR_FRAMES_TO_SAVE; i++) {
		ret = IMP_IVS_PollingResult(chn_num, IMP_IVS_DEFAULT_TIMEOUTMS);
		if (ret < 0) {
			IMP_LOG_ERR(TAG, "IMP_IVS_PollingResult(%d, %d) failed\n", chn_num, IMP_IVS_DEFAULT_TIMEOUTMS);
			return (void *)-1;
		}
		ret = IMP_IVS_GetResult(chn_num, (void **)&result);
		if (ret < 0) {
			IMP_LOG_ERR(TAG, "IMP_IVS_GetResult(%d) failed\n", chn_num);
			return (void *)-1;
		}
#ifdef DEBUG_PRINT
  //printf("======>: printf source datalen\n");
  m = 0; n = 0;
  flg = 0;
		for(j = 0; j < result->datalen; j ++) {
			//printf("%4d ",*(result->data + j));
			if(*(result->data + j) != 0x00)
			{
			  array[m][n] = 1;
			  flg = 1;
			}
			else
			{
			  array[m][n] = 0;
			}
			n++;
			if(j%(SENSOR_WIDTH_SECOND/SAD_MODE_SIZE) == 0)
			{
			  //printf("\n");
			  m++;
			  n = 0;
			}
		}
		
		//printf("\r\n");
#endif

#ifdef DEBUG_PRINT

  for(j = 0; j < 45; j++)
  {
    for(i = 0; i < 80; i++)
    {
      printf("%d,", array[j][i]);
    }
    printf("\r\n");
  }
		printf("\r\n");
  if(flg == 1)
  {
  		/* 计算出区域坐标 */
    findMinMaxCoordinates(array, &minRow, &minCol, &maxRow, &maxCol);  
     if (minRow != INT_MAX && minCol != INT_MAX) 
    {  
      //printf("Minimum coordinates: Row = %d, Col = %d\n", minRow, minCol);  
      //printf("Maximum coordinates: Row = %d, Col = %d\n", maxRow, maxCol);  
      printf("source:(%d, %d) (%d, %d)\n", minCol, minRow ,maxCol, maxRow);
  
      printRecflg = 1;
      DispCnt = 0;
      
      p00.x = minCol*8*3;
      p00.y = minRow*8*3;
  
      p01.x = maxCol*8*3;
      p01.y = maxRow*8*3;
      printf("scale:(%d, %d) (%d, %d)\n", minCol, minRow ,maxCol, maxRow);
    }
    else 
    {  
    }  
  }
  else
  {
    printRecflg = 0;
    DispCnt = 0;
  }
  
#endif

  

		ret = IMP_IVS_ReleaseResult(chn_num, (void *)result);
		if (ret < 0) {
			IMP_LOG_ERR(TAG, "IMP_IVS_ReleaseResult(%d) failed\n", chn_num);
			return (void *)-1;
		}
#if 0
		if (i % 20 == 0) {
			ret = sample_ivs_set_sense(chn_num, i % 5);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "sample_ivs_set_sense(%d, %d) failed\n", chn_num, i % 5);
				return (void *)-1;
			}
		}
#endif
	}

	return (void *)0;
}

static int sample_ivs_move_get_result_start(int chn_num, pthread_t *ptid)
{
	if (pthread_create(ptid, NULL, sample_ivs_move_get_result_process, (void *)chn_num) < 0) {
		IMP_LOG_ERR(TAG, "create sample_ivs_move_get_result_process failed\n");
		return -1;
	}

	return 0;
}

static int sample_ivs_move_get_result_stop(pthread_t tid)
{
	pthread_join(tid, NULL);
	return 0;
}



static int osd_show(void)
{
	int ret;

	ret = IMP_OSD_ShowRgn(prHander[0], grpNum, 1);
	if (ret != 0) {
		IMP_LOG_ERR(TAG, "IMP_OSD_ShowRgn() timeStamp error\n");
		return -1;
	}
	ret = IMP_OSD_ShowRgn(prHander[1], grpNum, 1);
	if (ret != 0) {
		IMP_LOG_ERR(TAG, "IMP_OSD_ShowRgn() Logo error\n");
		return -1;
	}

	return 0;
}

static void *update_thread(void *p)
{
	int ret;

	/*generate time*/
	char DateStr[40];
	time_t currTime;
	struct tm *currDate;
	unsigned i = 0, j = 0;
	void *dateData = NULL;
	uint32_t *data = p;
	IMPOSDRgnAttrData rAttrData;

	ret = osd_show();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "OSD show error\n");
		return NULL;
	}

	while(1) {
			int penpos_t = 0;
			int fontadv = 0;

			time(&currTime);
			currDate = localtime(&currTime);
			memset(DateStr, 0, 40);
			strftime(DateStr, 40, "%Y-%m-%d %I:%M:%S", currDate);
			for (i = 0; i < OSD_LETTER_NUM; i++) {
				switch(DateStr[i]) {
					case '0' ... '9':
						dateData = (void *)gBgramap[DateStr[i] - '0'].pdata;
						fontadv = gBgramap[DateStr[i] - '0'].width;
						penpos_t += gBgramap[DateStr[i] - '0'].width;
						break;
					case '-':
						dateData = (void *)gBgramap[10].pdata;
						fontadv = gBgramap[10].width;
						penpos_t += gBgramap[10].width;
						break;
					case ' ':
						dateData = (void *)gBgramap[11].pdata;
						fontadv = gBgramap[11].width;
						penpos_t += gBgramap[11].width;
						break;
					case ':':
						dateData = (void *)gBgramap[12].pdata;
						fontadv = gBgramap[12].width;
						penpos_t += gBgramap[12].width;
						break;
					default:
						break;
				}
#ifdef SUPPORT_RGB555LE
				for (j = 0; j < OSD_REGION_HEIGHT; j++) {
					memcpy((void *)((uint16_t *)data + j*OSD_LETTER_NUM*OSD_REGION_WIDTH + penpos_t),
							(void *)((uint16_t *)dateData + j*fontadv), fontadv*sizeof(uint16_t));
				}
#else
				for (j = 0; j < OSD_REGION_HEIGHT; j++) {
					memcpy((void *)((uint32_t *)data + j*OSD_LETTER_NUM*OSD_REGION_WIDTH + penpos_t),
							(void *)((uint32_t *)dateData + j*fontadv), fontadv*sizeof(uint32_t));
				}

#endif
			}
			rAttrData.picData.pData = data;
			IMP_OSD_UpdateRgnAttrData(prHander[0], &rAttrData);


			
   IMPOSDRgnAttr rAttrRect;
   memset(&rAttrRect, 0, sizeof(IMPOSDRgnAttr));


   if(printRecflg == 1)
   {
     rAttrRect.type = OSD_REG_RECT;
    
     rAttrRect.rect.p0.x = p00.x;
     rAttrRect.rect.p0.y = p00.y;
     rAttrRect.rect.p1.x = p01.x;
     rAttrRect.rect.p1.y = p01.x;
     rAttrRect.fmt = PIX_FMT_MONOWHITE;
     rAttrRect.data.lineRectData.color = OSD_RED;
     rAttrRect.data.lineRectData.linewidth = 5;
     ret = IMP_OSD_SetRgnAttr(prHander[1], &rAttrRect);
     if (ret < 0) {
      IMP_LOG_ERR(TAG, "IMP_OSD_SetRgnAttr Rect error !\n");
      return NULL;
     }

     DispCnt++;
     if(DispCnt >= 20)
     {
      printRecflg = 0;
      DispCnt = 0;
    }
   }
   else
   {
     rAttrRect.type = OSD_REG_RECT;
    
     rAttrRect.rect.p0.x = 0;
     rAttrRect.rect.p0.y = 0;
     rAttrRect.rect.p1.x = 1;
     rAttrRect.rect.p1.y = 1;
     rAttrRect.fmt = PIX_FMT_MONOWHITE;
     rAttrRect.data.lineRectData.color = OSD_RED;
     rAttrRect.data.lineRectData.linewidth = 5;
     ret = IMP_OSD_SetRgnAttr(prHander[1], &rAttrRect);
     if (ret < 0) {
      IMP_LOG_ERR(TAG, "IMP_OSD_SetRgnAttr Rect error !\n");
      return NULL;
     }
   }

			usleep(500*1000);
	}

	return NULL;
}


int main(int argc, char *argv[])
{
	int i, ret;
	pthread_t ivs_tid;
	IMPIVSInterface *inteface = NULL;

	/* Step.1 System init */
	ret = sample_system_init();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "IMP_System_Init() failed\n");
		return -1;
	}

	/* Step.2 FrameSource init */
	ret = sample_framesource_init();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource init failed\n");
		return -1;
	}

	/* Step.3 Encoder init */
	for (i = 0; i < FS_CHN_NUM; i++) {
		if (chn[i].enable) {
			ret = IMP_Encoder_CreateGroup(chn[i].index);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "IMP_Encoder_CreateGroup(%d) error !\n", i);
				return -1;
			}
		}
	}

	ret = sample_encoder_init();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "Encoder init failed\n");
		return -1;
	}

	
  if (IMP_OSD_CreateGroup(grpNum) < 0) {
   IMP_LOG_ERR(TAG, "IMP_OSD_CreateGroup(%d) error !\n", grpNum);
   return -1;
  }
 
  /* Step.4 OSD init */
  prHander = sample_osd_init(grpNum);
  if (prHander <= 0) {
   IMP_LOG_ERR(TAG, "OSD init failed\n");
   return -1;
  }
 
  /* Step.5 Bind */
  IMPCell osdcell = {DEV_ID_OSD, grpNum, 0};
  ret = IMP_System_Bind(&chn[0].framesource_chn, &osdcell);
  if (ret < 0) {
   IMP_LOG_ERR(TAG, "Bind FrameSource channel0 and OSD failed\n");
   return -1;
  }
 
  ret = IMP_System_Bind(&osdcell, &chn[0].imp_encoder);
  if (ret < 0) {
   IMP_LOG_ERR(TAG, "Bind OSD and Encoder failed\n");
   return -1;
  }
 
  /* Step.6 Create OSD bgramap update thread */
  pthread_t tid;
#ifdef SUPPORT_RGB555LE
  uint32_t *timeStampData = malloc(OSD_LETTER_NUM * OSD_REGION_HEIGHT * OSD_REGION_WIDTH * sizeof(uint16_t));
#else
  uint32_t *timeStampData = malloc(OSD_LETTER_NUM * OSD_REGION_HEIGHT * OSD_REGION_WIDTH * sizeof(uint32_t));
#endif
  if (timeStampData == NULL) {
   IMP_LOG_ERR(TAG, "valloc timeStampData error\n");
   return -1;
  }
 
  ret = pthread_create(&tid, NULL, update_thread, timeStampData);
  if (ret) {
   IMP_LOG_ERR(TAG, "thread create error\n");
   return -1;
  }

	/* Step.4 ivs init */
	ret = sample_ivs_move_init(0);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_init(0) failed\n");
		return -1;
	}

    /* step.5 bind */

	/* bind framesource channel.1-output.1 to ivs group */
	/**
	 * fs.0 ----------------> encoder.0(main stream)
	 * fs.1 ----------------> ivs----> encoder.1(second stream)
	 */
	IMPCell ivs_cell = {DEV_ID_IVS, 0, 0};
	IMPCell fs_for_ivs_cell = {DEV_ID_FS, 2, 2};
    for (i = 0; i < FS_CHN_NUM; i++) {
        if (IVS_CHN_ID == i) {
            if (chn[i].enable) {

                ret = IMP_System_Bind(&chn[i].framesource_chn, &ivs_cell);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "Bind FrameSource channel.1 output.1 and ivs0 failed\n");
                    return -1;
                }
                ret = IMP_System_Bind(&ivs_cell, &chn[i].imp_encoder);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "Bind FrameSource channel%d and Encoder failed\n",i);
                    return -1;
                }
            }
        } else {
            if (chn[i].enable) {
                ret = IMP_System_Bind(&chn[i].framesource_chn, &chn[i].imp_encoder);
                if (ret < 0) {
                    IMP_LOG_ERR(TAG, "Bind FrameSource channel%d and Encoder failed\n",i);
                    return -1;
                }
            }
        }
    }

    /* Step.6 framesource Stream On */
	ret = sample_framesource_streamon();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "ImpStreamOn failed\n");
		return -1;
	}

	/* Step.7 ivs move start */
	ret = sample_ivs_move_start(0, 2, &inteface);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_start(0, 0) failed\n");
		return -1;
	}

	/* Step.8 start to get ivs move result */
	ret = sample_ivs_move_get_result_start(2, &ivs_tid);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_get_result_start failed\n");
		return -1;
	}

	/* Step.9 get h264 stream */
	ret = sample_get_video_stream();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "Get H264 stream failed\n");
		return -1;
	}

	/* Exit sequence as follow */

	/* Step.10 stop to get ivs move result */
	ret = sample_ivs_move_get_result_stop(ivs_tid);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_get_result_stop failed\n");
		return -1;
	}

	/* Step.11 ivs move stop */
	ret = sample_ivs_move_stop(2, inteface);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_ivs_move_stop(0) failed\n");
		return -1;
	}

	/* Step.12 Stream Off */
	ret = sample_framesource_streamoff();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource StreamOff failed\n");
		return -1;
	}

	/* Step.13 UnBind */
	ret = IMP_System_UnBind(&chn[IVS_CHN_ID].framesource_chn, &ivs_cell);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "UnBind FrameSource channel%d and ivs0 failed\n", IVS_CHN_ID);
		return -1;
	}

	for (i = 0; i < FS_CHN_NUM; i++) {
		if (chn[i].enable) {
			if(IVS_CHN_ID == i) {
				ret = IMP_System_UnBind(&ivs_cell, &chn[i].imp_encoder);
				if (ret < 0) {
					IMP_LOG_ERR(TAG, "UnBind FrameSource channel%d and Encoder failed\n",i);
					return -1;
				}
			}else{
			ret = IMP_System_UnBind(&chn[i].framesource_chn, &chn[i].imp_encoder);
			if (ret < 0) {
				IMP_LOG_ERR(TAG, "UnBind FrameSource channel%d and Encoder failed\n",i);
				return -1;
			}
			}
		}
	}

	/* Step.14 ivs exit */
	ret = sample_ivs_move_exit(0);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "ivs mode exit failed\n");
		return -1;
	}

		/* Step.c OSD exit */
	ret = sample_osd_exit(prHander,grpNum);
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "OSD exit failed\n");
		return -1;
	}

	/* Step.15 Encoder exit */
	ret = sample_encoder_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "Encoder exit failed\n");
		return -1;
	}

	/* Step.16 FrameSource exit */
	ret = sample_framesource_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "FrameSource exit failed\n");
		return -1;
	}

	/* Step.17 System exit */
	ret = sample_system_exit();
	if (ret < 0) {
		IMP_LOG_ERR(TAG, "sample_system_exit() failed\n");
		return -1;
	}

	return 0;
}
