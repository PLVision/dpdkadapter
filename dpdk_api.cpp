/*-
 * @file dpdk_api.cpp
 *
 * The file contains definition of C wrappers supposed to be used by external components.
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

#include "dpdk_api.h"
#include "dpdk_adapter.h"
#include "dpdk_log.h"

int dpdk_init(int& argc, char**& argv)
{
    return DPDKAdapter::instance()->init(argc, argv);
}

int dpdk_init_port(unsigned char devId)
{
    return DPDKAdapter::instance()->initPort(devId);
}

unsigned char dpdk_get_port_count()
{
    return DPDKAdapter::instance()->getPortsCount();
}

char* dpdk_get_dev_name(unsigned char devId, char* name)
{
    return DPDKAdapter::instance()->getDevName(devId, name);
}

void dpdk_reset_dev_stats(unsigned char devId)
{
    return DPDKAdapter::instance()->resetDevStats(devId);
}

int dpdk_set_dev_promisc(unsigned char devId, int promisc)
{
    return DPDKAdapter::instance()->setDevPromisc(devId, promisc);
}

int dpdk_start_dev(unsigned char devId)
{
    return DPDKAdapter::instance()->startDev(devId);
}

void dpdk_stop_dev(unsigned char devId)
{
    DPDKAdapter::instance()->stopDev(devId);
}

int dpdk_start_tx(unsigned char devId)
{
    DPDKAdapter::instance()->startTx(devId);
}

int dpdk_stop_tx(unsigned char devId)
{
    DPDKAdapter::instance()->stopTx(devId);
}

int dpdk_start_rx(unsigned char devId, char *captureData, unsigned int captureDataLength)
{
    DPDKAdapter::instance()->startRx(devId, captureData, captureDataLength);
}

int dpdk_stop_rx(unsigned char devId, unsigned int *pCaptureDataSize)
{
    DPDKAdapter::instance()->stopRx(devId, pCaptureDataSize);
}

int dpdk_add_packet_stream(unsigned char devId, unsigned int delay, unsigned int numPackets, unsigned int* pStreamId)
{
    DPDKAdapter::instance()->addPacketStream(devId, delay, numPackets, pStreamId);
}

int dpdk_add_burst_stream(unsigned char devId, unsigned int burstDelay, unsigned int burstSize, unsigned int numBursts, unsigned int* pStreamId)
{
    DPDKAdapter::instance()->addBurstStream(devId, burstDelay, burstSize, numBursts, pStreamId);
}

int dpdk_add_packet(unsigned char devId, unsigned int streamId, const void* data, unsigned int dataLen)
{
    DPDKAdapter::instance()->addPacket(devId, streamId, data, dataLen);
}

int dpdk_clear_packets(unsigned char devId)
{
    DPDKAdapter::instance()->clearPacketList(devId);
}

void dpdk_set_loop_mode(unsigned char devId, int loop)
{
    DPDKAdapter::instance()->setPacketListLoopMode(devId, loop);
}

void dpdk_set_mix_mode(unsigned char devId, dpdk_tx_mode_t mode)
{
    DPDKAdapter::instance()->setPacketListMixMode(devId, mode);
}

int dpdk_get_dev_link_status(unsigned char devId, int* pLinkStatus)
{
    EthLinkState_t linkState;
    int res = DPDKAdapter::instance()->getDevLinkState(devId, &linkState);

    *pLinkStatus = linkState.link_status;

    return res;
}

void dpdk_get_dev_stats(unsigned char devId, dpdk_stats_t* pStats)
{
    EthDevStatistics_t stats;
    DPDKAdapter::instance()->getDevStatistics(devId, &stats);

    pStats->ipackets = stats.ipackets;
    pStats->opackets = stats.opackets;
    pStats->ibytes = stats.ibytes;
    pStats->obytes = stats.obytes;

    pStats->imissed = stats.imissed;
    if (pStats->imissed)
        qCritical("Port %u: RX missed packets %llu", devId, pStats->imissed);

    pStats->ierrors = stats.ierrors;
    if (pStats->ierrors)
        qCritical("Port %u: Erroneous received packets %llu", devId, pStats->ierrors);

    pStats->ibadlen = stats.ibadlen;
    if (pStats->ibadlen)
        qCritical("Port %u: RX packets with bad length %llu", devId, pStats->ibadlen);

    pStats->ibadcrc = stats.ibadcrc;
    if (pStats->ibadcrc)
        qCritical("Port %u: RX packets with CRC error %llu", devId, pStats->ibadcrc);

    pStats->rx_nombuf = stats.rx_nombuf;
    if (pStats->rx_nombuf)
        qCritical("Port %u: RX mbuf allocation failures %llu", devId, pStats->rx_nombuf);
}
