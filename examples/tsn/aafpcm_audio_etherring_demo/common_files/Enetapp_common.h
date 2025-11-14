#ifndef __ENETAPP_COMMON_H_
#define __ENETAPP_COMMON_H_

#define GPTP_SYNC_WAIT_TIME_SEC  (10)
typedef enum
{
    NODE_INVALID = -1,
    NODE_CENTRAL,
    NODE_LEFT,
    NODE_RIGHT,
    NODE_TAIL,
    NODE_TOTAL_NODES,
}node_index;

typedef struct
{
    const char* nodeName;
}node_info;

int32_t Board_CdceConfig(void);

int32_t Board_codecConfig(void);

int32_t Board_MuxSelMcASP4(void);

void EnetApp_configControlData(void* args);

int32_t TsMcasp_tsMcaspConfig(int instIdx);

void aaf_audio_task(void *args);

void EnetApp_AudioPlaybackDemoMain(void* args);

int32_t EnetApp_addMcastEntry(uint8_t *mcastAddr, uint8_t portMask, uint16_t vlanId);

void aafTask_signalCrfStart(void);

void EnetApp_printStats(uint64_t currentTime);

extern node_index gNodeIndex;

#endif /* __ENETAPP_COMMON_H_ */