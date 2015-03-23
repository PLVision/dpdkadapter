/*-
 * @file dpdk_api.cpp
 *
 * The file contains declaration of C wrappers supposed to be used by external components.
 *
 * The file is part of DPDKAdapter project which release under proprietary license.
 * See the License file in the root of project.
 *
 * PLVISION CONFIDENTIAL
 * __________________
 *
 * Copyright (c) [2015] PLVISION sp. z o.o.
 * <developers@plvision.eu>
 * All Rights Reserved.
 */

#ifndef DPDK_ADAPTER_API_H
#define DPDK_ADAPTER_API_H

typedef struct dpdk_rx_cb_s
{
   void *pObj;
   void *pData;
   unsigned int dataLen;
   unsigned long long timestamp;
} dpdk_rx_cb_t;

typedef void (*dpdk_rx_callback_t)(dpdk_rx_cb_s cb_data);

typedef enum
{
    DPDK_SEQ_MODE,
    DPDK_INTER_MODE
} dpdk_tx_mode_t;

typedef struct dpdk_stats_s
{
    unsigned long long ipackets;
    unsigned long long ibytes;
    unsigned long long imissed;
    unsigned long long ierrors;
    unsigned long long ibadlen;
    unsigned long long ibadcrc;
    unsigned long long rx_nombuf;
    unsigned long long opackets;
    unsigned long long obytes;
} dpdk_stats_t;

extern "C"
{
  int dpdk_init(int& argc, char**& argv);
  int dpdk_init_port(unsigned char devId);
  unsigned char dpdk_get_port_count();
  char* dpdk_get_dev_name(unsigned char devId, char* name);
  void dpdk_reset_dev_stats(unsigned char devId);

  int dpdk_set_dev_promisc(unsigned char devId, int promisc);

  int dpdk_start_dev(unsigned char devId);
  void dpdk_stop_dev(unsigned char devId);

  void dpdk_assign_dev_obj(unsigned char devId, void* devRxObj);
  void dpdk_set_dev_rx_cb(dpdk_rx_callback_t cb);

  int dpdk_start_tx(unsigned char devId);
  int dpdk_stop_tx(unsigned char devId);

  int dpdk_start_rx(unsigned char devId, char *captureData, unsigned int captureDataLength);
  int dpdk_stop_rx(unsigned char devId, unsigned int *pCaptureDataSize);

  int dpdk_add_packet_stream(unsigned char devId, unsigned int delay, unsigned int numPackets, unsigned int* pStreamId);
  int dpdk_add_burst_stream(unsigned char devId, unsigned int burstDelay, unsigned int burstSize, unsigned int numBursts, unsigned int* pStreamId);
  int dpdk_add_packet(unsigned char devId, unsigned int streamId, const void* data, unsigned int dataLen);
  int dpdk_clear_packets(unsigned char devId);
  void dpdk_set_loop_mode(unsigned char devId, int loop);
  void dpdk_set_mix_mode(unsigned char devId, dpdk_tx_mode_t mode);

  int dpdk_get_dev_link_status(unsigned char devId, int* pLinkStatus);
  void dpdk_get_dev_stats(unsigned char devId, dpdk_stats_t* pStats);
}

#endif // DPDK_ADAPTER_API_H
