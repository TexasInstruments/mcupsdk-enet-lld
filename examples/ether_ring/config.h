/* Each ClassA Stream Task send 1 Class-A Stream */
#define NUM_CLASSA_STREAM_TASKS 1

/* Each ClassD Stream Task send 3 Class-D Stream */
#define NUM_CLASSD_STREAM_TASKS 1

/* Total ClassA Stream Count */
#define NUM_CLASSA_STREAMS NUM_CLASSA_STREAM_TASKS

/* Total ClassD Stream Count */
#define NUM_CLASSD_STREAMS (3*NUM_CLASSD_STREAM_TASKS)

/* Maximum Class-A Streams supported in the Application */
#define MAX_CLASSA_STREAMS 3

/* Maximum Class-D Streams supported in the Application */
#define MAX_CLASSD_STREAMS 3

/* ClassA Stream Payload Length */
#define CLASSA_PAYLOAD_LENGTH 1000U

/* ClassD Stream Payload Length */
#define CLASSD_PAYLOAD_LENGTH 1450U

/* Node Count present in Ether-Ring Topology */
#define NODES_COUNT_IN_ETHERRING 4

/* Macro to Enable and Disable Profiling for Ether-Ring */
#define ETHERRING_PROFILING

/* Enable Profiling for Class A with StreamId - 0 */
#define ETHERRING_PROFILE_STREAMID 0

/* Hardware Timer periodicity configured from syscfg */
#define TIMER_CB_PERIODICITY 125*UB_USEC_NS

/* ClassA Stream Periodicity 125us */
#define CLASSA_STREAM_TRAFFIC_PERIODICITY TIMER_CB_PERIODICITY

/* ClassD Stream Periodicity 1000us */
#define CLASSD_STREAM_TRAFFIC_PERIODICITY (8 * CLASSA_STREAM_TRAFFIC_PERIODICITY)

/* EST Enable/Disable */
#define ENABLE_EST

#ifdef ETHERRING_PROFILING
/*! \brief Value of Maximum Rx TimeStamps stored in the EtherRing RxTs object */
#define ETHERRINGAPP_MAX_RX_TIMESTAMPS_STORED                                    30U
#endif
