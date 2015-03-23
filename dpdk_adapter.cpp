/*-
 * @file dpdk_adapter.cpp
 *
 * The file contains classes definition to work with DPDK library.
 * Init/Deinit functionality is used to properly initialize DPDK lib.
 * RX/TX routine is created to handle incoming and out-coming packets.
 * StreamInfo an PacketInfo classes are used to store stream related data.
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

#include "dpdk_adapter.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/queue.h>
#include <netinet/in.h>
#include <setjmp.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>

#include <rte_pci.h>
#include <rte_common.h>
#include <rte_log.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_tailq.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_launch.h>
#include <rte_atomic.h>
#include <rte_cycles.h>
#include <rte_prefetch.h>
#include <rte_lcore.h>
#include <rte_per_lcore.h>
#include <rte_branch_prediction.h>
#include <rte_interrupts.h>
#include <rte_pci.h>
#include <rte_random.h>
#include <rte_debug.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_ring.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_malloc.h>

#include "dpdk_log.h"
#include <pcap.h>

#define SEC_TO_NSEC 1E9
#define MSEC_TO_NSEC 1e3

#define MAX_SEG_SIZE 2048

#define MAX_PACKET_SIZE 10024

#define MBUF_SIZE (MAX_SEG_SIZE + sizeof(struct rte_mbuf) + RTE_PKTMBUF_HEADROOM)

#define DPDKADAPTER_TX_QUEUE_NUM 1
#define DPDKADAPTER_RX_QUEUE_NUM 1

#define TX_         "tx_"
#define RX_         "rx_"

/**
 * @brief           Parse a value saved in a file
 *
 * @param filename  Full file name
 * @param val       Value stored in the file
 *
 * @return          0 on success
 * @return          -1 on failure
 */
int parse_sysfs_value(const char *filename, unsigned long *val)
{
	FILE *f;
	char buf[BUFSIZ];
	char *end = NULL;

	if ((f = fopen(filename, "r")) == NULL) {
		return -1;
	}

	if (fgets(buf, sizeof(buf), f) == NULL) {
		fclose(f);
		return -1;
	}
	*val = strtoul(buf, &end, 0);
	if ((buf[0] == '\0') || (end == NULL) || (*end != '\n')) {
		fclose(f);
		return -1;
	}
	fclose(f);
	return 0;
}

DPDKAdapter* DPDKAdapter::instance_ = NULL;

DPDKProfiler::StatsMap_t DPDKProfiler::stats_[RTE_MAX_LCORE] = {};

/**
 * @brief           DPDKProfiler constructor
 *
 * @param coreId    Core number
 * @param name      Name of a profiled section
 *
 * @return          true on success
 */
DPDKProfiler::DPDKProfiler(uint8_t coreId, const char* name) : start_(0), coreId_(coreId), name_(name)
{
    start_ = rte_get_tsc_cycles();
}

/**
 * @brief           DPDKProfiler destructor
 */
DPDKProfiler::~DPDKProfiler()
{
    uint64_t end = rte_get_tsc_cycles();

    stats_[coreId_][name_].last_duration = (end - start_) * SEC_TO_NSEC / rte_get_tsc_hz();
    stats_[coreId_][name_].total_duration += stats_[coreId_][name_].last_duration;
    stats_[coreId_][name_].invoke_cnt += 1;

    if (stats_[coreId_][name_].invoke_cnt == 10000000)
    {
        qWarning("%s on core %u : last duration %lu, medium duration %lu", name_.c_str(), coreId_, lastDurationGet(coreId_, name_), medDurationGet(coreId_, name_));
        reset(coreId_, name_);
    }
}

/**
 * @brief           Get duration
 *
 * @param coreId    Core number
 * @param name      Name of a profiled section
 *
 * @return          Duration in nano seconds
 */
uint64_t DPDKProfiler::lastDurationGet(uint8_t coreId, std::string& name)
{
    return stats_[coreId][name].last_duration;
}

/**
 * @brief           Get median duration
 *
 * @param coreId    <description>
 * @param name      <description>
 *
 * @return          Duration in nano seconds
 */
uint64_t DPDKProfiler::medDurationGet(uint8_t coreId, std::string& name)
{
    uint64_t med_duration = 0;

    med_duration = stats_[coreId][name].total_duration / stats_[coreId][name].invoke_cnt;

    return med_duration;
}

/**
 * @brief           Get number of invocations
 *
 * @param coreId    Core number
 * @param name      Name of a profiled section
 *
 * @return          Number of invocations
 */
uint64_t DPDKProfiler::invokeCountGet(uint8_t coreId, std::string& name)
{
    return stats_[coreId][name].invoke_cnt;
}

/**
 * @brief           Reset all profiler counters
 *
 * @param coreId    Core number
 * @param name      Name of a profiled section
 *
 */
void DPDKProfiler::reset(uint8_t coreId, std::string& name)
{
    memset(&stats_[coreId][name], 0, sizeof(stats_[coreId]));
}

/**
 * @brief           DeviceInfo constructor
 */
DPDKAdapter::DeviceInfo::DeviceInfo() :
    txDevStart(false),
    rxDevStart(false),
    loopMode(false),
    iterationCnt(0),
    mode(DPDK_SEQ_MODE),
    rxQueueCount(0),
    curStream(0)
{
    rxData = (char*)malloc(MAX_PACKET_SIZE);
    if (rxData == NULL)
    {
        qCritical("Could not allocate a memory");
    }

    qDebug("rxData %p", rxData);
}

/**
 * @brief           DeviceInfo destructor
 */
DPDKAdapter::DeviceInfo::~DeviceInfo()
{
    qDebug("rxData %p", rxData);

    if (rxData)
    {
        free(rxData);
    }
}

/**
 * @brief           PacketInfo constructor
 *
 * @param devId     Port number
 * @param data      Packet data
 * @param dataLen   Packet data length
 */
DPDKAdapter::PacketInfo::PacketInfo(uint8_t devId, const void* data, unsigned int dataLen) :
    devId_(devId),
    mbuf_(NULL),
    data_(NULL),
    dataLen_(0)
{
    DPDKAdapter::instance()->copyBufToMbuf(devId, (char*)data, dataLen, mbuf_);

    qDebug("mbuf_ %p", mbuf_);

    data_ = (char*)rte_malloc("packet data", dataLen, 0);
   
    if (!data_)
    {
        qCritical("Could not allocate memory for a packet");
    }

    rte_memcpy(data_, data, dataLen);

    dataLen_ = dataLen;
}

/**
 * @brief           PacketInfo copy constructor
 *
 * @param other     Object to be copied
 */
DPDKAdapter::PacketInfo::PacketInfo(const PacketInfo& other)
{
    devId_ = other.devId_;
    mbuf_ = DPDKAdapter::instance()->cloneMbuf(devId_, other.mbuf_);

    data_ = (char*)rte_malloc("packet data", other.dataLen_, 0);

    if (!data_)
    {
        qCritical("Could not allocate memory for a packet");
    }

    rte_memcpy(data_, other.data_, other.dataLen_);
    dataLen_ = other.dataLen_;

    qDebug("mbuf_ %p", mbuf_);
}

/**
 * @brief           Get pointer to mbuf
 *
 * @return          A pointer to the mbuf member
 */
MBuf_t* DPDKAdapter::PacketInfo::getMbuf()
{
    return mbuf_;
}

/**
 * @brief           PacketInfo destructor
 *
 */
DPDKAdapter::PacketInfo::~PacketInfo()
{
    qDebug("mbuf_ %p", mbuf_);

    if (mbuf_)
    {
        DPDKAdapter::instance()->txMbufFree(mbuf_);
    }

    if (data_)
    {
        rte_free(data_);
    }
}

/**
 * @brief             StreamInfo constructor
 *
 * @param devId       Port number
 * @param delay       Delay in nano seconds
 * @param burstSize   Stream burst size
 * @param numBursts   Number of bursts
 * @param txBurstSize A minimal packets batch size put on the wire
 */
DPDKAdapter::StreamInfo::StreamInfo(unsigned char devId, unsigned int delay, unsigned int burstSize, unsigned int numBursts, uint8_t txBurstSize) :
    devId_(devId),
    delay_(delay),
    lastTx_(0),
    numBursts_(numBursts),
    burstSize_(burstSize),
    txBurstSize_(txBurstSize),
    sentPackets_(0),
    sentBursts_(0),
    curPacketId_(0)
{
    ticksDelay_ = (uint64_t)delay * rte_get_tsc_hz() / SEC_TO_NSEC;

    qDebug("txBurstSize %u", txBurstSize); 

    qDebug("ticksDelay_ %llu, CPU frequency %llu", ticksDelay_, rte_get_tsc_hz());
}

/**
 * @brief           StreamInfo copy constructor
 *
 * @param other     Object to be copied
 *
 */
DPDKAdapter::StreamInfo::StreamInfo(const StreamInfo& other)
{
    devId_ = other.devId_;
    delay_ = other.delay_;
    lastTx_ = other.lastTx_;
    numBursts_ = other.numBursts_;
    txBurstSize_ = other.txBurstSize_;
    burstSize_ = other.burstSize_;
    sentPackets_ = other.sentPackets_;
    sentBursts_ = other.sentBursts_;
    curPacketId_ = other.curPacketId_;
    ticksDelay_ = (uint64_t)delay_ * rte_get_tsc_hz() / SEC_TO_NSEC;

    packets_ = other.packets_;

    qDebug("ticksDelay_ %llu, CPU frequency %llu", ticksDelay_, rte_get_tsc_hz());
}

/**
 * @brief           StreamInfo destructor
 *
 */
DPDKAdapter::StreamInfo::~StreamInfo()
{
}

/**
 * @brief           Add a packet to the stream
 *
 * @param data      Packet sata
 * @param dataLen   Packet data length
 *
 */
void DPDKAdapter::StreamInfo::addPacket(const void* data, unsigned int dataLen)
{
    PacketInfo packet(devId_, data, dataLen);
    packets_.push_back(packet);

    qDebug("dataLen %u", dataLen);
}

/**
 * @brief              Get next packet from the stream
 *
 * @param curPacketId  A packet identifier
 *
 * @return             Next packet identifier
 */
unsigned int DPDKAdapter::StreamInfo::getNextPacketId(unsigned int curPacketId)
{
    if (curPacketId >= packets_.size() - 1)
    {
        return 0;
    }

    curPacketId++;

    qDebug("curPacketId %u", curPacketId);

    return curPacketId;
}

/**
 * @brief           Get a pointer to mbuf
 *
 * @param packetId  Packet identifier
 * @param pMBuf     Reference to mbuf pointer
 *
 * @return          true on success
 */
bool DPDKAdapter::StreamInfo::getMbuf(unsigned int packetId, MBuf_t*& pMBuf)
{
    if (packetId >= packets_.size())
    {
        return false;
    }

    pMBuf = packets_[packetId].getMbuf();

    return true;
}

/**
 * @brief           Get a packet info
 *
 * @param packetId  Packet identifier
 * @param pPacket   A reference to packet info
 *
 * @return          true on success
 */
bool DPDKAdapter::StreamInfo::getPacket(unsigned int packetId, PacketInfo*& pPacket)
{
    if (packetId >= packets_.size())
    {
        return false;
    }

    pPacket = &packets_[packetId];

    return true;
}

/**
 * @brief           Check if it is time to transmit
 *
 * @return          true if time is up
 */
bool DPDKAdapter::StreamInfo::isReadyTransmit()
{
    uint64_t currentTicks = rte_get_tsc_cycles();

    if ((lastTx_ == 0) || (currentTicks - lastTx_ >= ticksDelay_))
    {
        lastTx_ = currentTicks;
        return true;
    }

    return false;
}

/**
 * @brief           Reset transmit timer
 */
void DPDKAdapter::StreamInfo::resetTransmitTimer()
{
    lastTx_ = 0;
}

/**
 * @brief           Check if all bursts have been transmitted
 */
bool DPDKAdapter::StreamInfo::allBurstsTransmitted()
{
    bool allTransmitted =  sentBursts_ >= numBursts_;

    return allTransmitted;
}

/**
 * @brief           Reset number of sent packets
 */
void DPDKAdapter::StreamInfo::resetPktCnt()
{
    sentPackets_ = 0;
}

/**
 * @brief           Reset number of sent bursts
 */
void DPDKAdapter::StreamInfo::resetBurstCnt()
{
    sentBursts_ = 0;
}

/**
 * @brief           Pause for a requested time in ns
 */
void DPDKAdapter::StreamInfo::nPause()
{
    const uint64_t start = rte_get_tsc_cycles();

    while ((rte_get_tsc_cycles() - start) < ticksDelay_)
    {
        rte_pause();
    }
}

/**
 * @brief           Send packets in a burst
 *
 * @param devId     ID of DPDK device
 *
 * @return          true on success
 */
bool DPDKAdapter::StreamInfo::sendBurst(uint8_t devId)
{
    PacketInfo  *pPacket   = NULL;
    uint32_t    txPktCnt   = 0;
    uint8_t     txPktIndex = 0;
    MBuf_t*     txBurstBuf[DPDK_TX_MAX_PKT_BURST];

    while (txPktCnt < txBurstSize_)
    {
        for (int i = 0; i < burstSize_; i++)
        {
            txPktIndex = txPktCnt % txBurstSize_;

            if (!getMbuf(curPacketId_, txBurstBuf[txPktIndex]))
            {
                qCritical("Could not get mbuf");
                continue;
            }

            if (txPktIndex == txBurstSize_ - 1)
            {
                uint8_t txSentPkt = DPDKAdapter::instance()->sendMbufBurstWithoutFree(devId, txBurstBuf, txBurstSize_);

                if (txSentPkt == txBurstSize_)
                {
                    if (DPDKAdapter::instance()->isRxStarted(devId))
                    {
                        DPDKAdapter::instance()->saveToBuf(devId, txBurstBuf, txBurstSize_);
                    }
                }
            }

            curPacketId_ = getNextPacketId(curPacketId_);

            txPktCnt++;
            sentPackets_++;
        }

        sentBursts_++;
    }

    return true;
}

/**
 * @brief           Initialize DPDK Adapter instance
 * 
 * @return          Pointer to DPDK Adapter object
 */
DPDKAdapter* DPDKAdapter::instance()
{
    if(!instance_)
    {
        instance_ = new DPDKAdapter;
    }

    return instance_;
}

/**
 * @brief           Destroy DPDK Adapter instance
 */
void DPDKAdapter::destroy()
{
    if(instance_)
    {
        delete instance_;
        instance_ = NULL;
    }
    
    qDebug("DPDK Adapter instance has been destroyed");
}

/**
 * @brief           DPDK Adapter constructor
 */
DPDKAdapter::DPDKAdapter() :
             nPortCount(0),
             initialized(false),
             txGlobalStart(false),
             rxGlobalStart(false)
{
}

/**
 * @brief           DPDK Adapter destructor
 */
DPDKAdapter::~DPDKAdapter()
{
    stopTx();
    stopRx();

    rte_eal_mp_wait_lcore();
}

/**
 * @brief           Initialize DPDK related functionality
 *
 * @param argc      Number of command line arguments
 * @param argv      Command line arguments array
 * @param ret       Number of processed command line arguments
 *
 * @return          true if success and false otherwice
 */
bool DPDKAdapter::init(int& argc, char**& argv)
{
    qDebug("Initializing DPDK");

    long unsigned int nr_hugepages = 0;
    parse_sysfs_value("/proc/sys/vm/nr_hugepages", &nr_hugepages);
    if(nr_hugepages == 0)
    {
        qCritical("No huge pages found");
        return false;
    }

    int ret = rte_eal_init(argc, argv);
    if(ret < 0)
    {
        qCritical("Invalid EAL arguments");
        return false;
    }
    
    argc -= ret;
    argv += ret;

    if(!initializeDevs())
    {
        qCritical("Could not initialize devices");
        return false;
    }

    portTxCoreMap(0, 1);
    portTxCoreMap(1, 2);

    portRxCoreMap(0, 1);
    portRxCoreMap(1, 2);

    startTx();
    startRx();

    rte_eal_mp_remote_launch(lcoreMainRoutine, this, SKIP_MASTER);

    initialized = true;

    return true;
}

/**
 * @brief           Check if adapter is initialized
 *
 * @return          true if it is initialized
 */
bool DPDKAdapter::isInitialized()
{
    return initialized;
}

/**
 * @brief           Map port RX to a logical core
 *
 * @param devId     Port number
 * @param coreId    Core number
 *
 * @return          true on success
 */
bool DPDKAdapter::portRxCoreMap(uint8_t devId, uint8_t coreId)
{
    if (devId > RTE_MAX_ETHPORTS)
    {
        qCritical("devId %u is not supported", devId);
        return false;
    }

    if (!rte_lcore_is_enabled(coreId))
    {
        qCritical("lcore %u is not enabled", coreId);
        return false;
    }

    LcoreInfo& coreInfo = cores[coreId];
    coreInfo.rxPortList.insert(devId);

    qWarning("RX port %u is mapped to lcore %u", devId, coreId);

    return true;
}

/**
 * @brief           Map port TX to a logical core
 *
 * @param devId     Port number
 * @param coreId    Core number
 *
 * @return          true on success
 */
bool DPDKAdapter::portTxCoreMap(uint8_t devId, uint8_t coreId)
{
    if (devId > RTE_MAX_ETHPORTS)
    {
        qCritical("devId %u is not supported", devId);
        return false;
    }

    if (!rte_lcore_is_enabled(coreId))
    {
        qCritical("lcore %u is not enabled", coreId);
        return false;
    }

    LcoreInfo& coreInfo = cores[coreId];
    coreInfo.txPortList.insert(devId);

    qWarning("TX port %u is mapped to lcore %u", devId, coreId);

    return true;
}

/**
 * @brief           Initialize all DPDK devices
 *
 * @return          true on success
 */
bool DPDKAdapter::initializeDevs()
{
    if(rte_eal_pci_probe() < 0)
    {
        qCritical("Cannot probe PCI");
        return false;
    }

    nPortCount = rte_eth_dev_count();
    if(nPortCount < 1)
    {
        qCritical("No ports found");
        return false;
    }

    //Limit devices count
    if(nPortCount > RTE_MAX_ETHPORTS)
        nPortCount = RTE_MAX_ETHPORTS;

    for(u_int8_t portId; portId < nPortCount; ++portId)
    {
        if(!initDevRxTxMPool(portId))
        {
            qCritical("TX/RX pools could not been allocated : port %u", portId);
            return false;
        }
    }    

    return true;
}

/**
 * @brief           Initialize DPDK device
 * 
 * @param devId     uint8_t, ID of DPDK device
 * 
 * @return          true if success and false otherwice
 */
bool DPDKAdapter::initPort(uint8_t devId)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return false;
    }

    if(0 > configureDev(devId, DPDKADAPTER_RX_QUEUE_NUM, DPDKADAPTER_TX_QUEUE_NUM))
        return false;

    if(0 > initRxQueue(devId, 0, DPDK_RX_DESC_NUMBER))
        return false;

    if(0 > initTxQueue(devId, 0, DPDK_TX_DESC_NUMBER))
        return false;

    return true;
}

/**
 * @brief           Initialize MBuf pool for device
 * 
 * @param devId     uint8_t, device identifier
 * 
 * @return          true if success and false otherwice
 */
bool DPDKAdapter::initDevRxTxMPool(unsigned int devId)
{
    char name[RTE_MEMPOOL_NAMESIZE];
    
    mbufPoolNameBuilder(devId, name, sizeof(name), TX_);
    if(!initDevMBufPool(name))
        return false;
    
    mbufPoolNameBuilder(devId, name, sizeof(name), RX_);
    if(!initDevMBufPool(name))
        return false;
    
    return true;
}

/**
 * @brief           Initialize MBuf pool for device
 * 
 * @param name      const char*, name of MemPool object
 * 
 * @return          true if success and false otherwice
 */
bool DPDKAdapter::initDevMBufPool(const char* name)
{
    if(!name)
        return false;
        
    // Don't create MemPool if it already exists
    MemPool_t* pool = rte_mempool_lookup(name);
    if(pool)
        return pool;
    
    pool = rte_mempool_create(name,
                DPDK_MEMPOOL_SIZE,
                MBUF_SIZE,
                DPDK_MEMPOOL_CACHE_SIZE,
                sizeof(struct rte_pktmbuf_pool_private),
                rte_pktmbuf_pool_init, NULL,
                rte_pktmbuf_init, NULL,
                SOCKET_ID_ANY,
                MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET);

    if(pool == NULL)
    {
        qCritical("Can not init memory pool");
        return false;
    }

    if(rte_mempool_lookup(name) != pool)
    {
        qCritical("Can not lookup memory pool by its name");
        return false;
    }

    return true;
}

/**
 * @brief           Build a name for a mempool
 *
 */
void DPDKAdapter::mbufPoolNameBuilder(unsigned int devId, char* name, int nameLen, const char* prefix)
{
    snprintf(name, nameLen, "%smbuf_pool_%u", prefix, devId);
}

/**
 * @brief           Lookup for MBuf assigned to socket
 * 
 * @param devId     uint8_t, device identifier
 * @param prefix    const char*, prefix for MemPool name
 * 
 * @return          MBuf_t*, pointer to MBuf that was found
 */
MemPool_t* DPDKAdapter::findMPool(uint8_t devId, const char* prefix)
{
    char name[RTE_MEMPOOL_NAMESIZE];

    mbufPoolNameBuilder(devId, name, sizeof(name), prefix);
    
    return (rte_mempool_lookup((const char*)name));
}

/**
 * @brief           Retrieve device information
 * 
 * @param devId     uint8_t, ID of DPDK device
 * @param info      EthDevInfo_t*, pointer to buffer where device info will be stored
 *
 * @return          EthDevInfo_t* if success and NULL otherwice
 */
EthDevInfo_t* DPDKAdapter::getDevInfo(uint8_t devId, EthDevInfo_t* info)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return NULL;
    }

    memset(info, 0, sizeof(EthDevInfo_t));
    rte_eth_dev_info_get(devId, info);
    
    return info;
}

/**
 * @brief           Retrieve device name
 * 
 * @param devId     uint8_t, ID of DPDK device
 * @param name      char*, pointer to buffer where device name will be stored
 *
 * @return          char* pointer to buffer where device name is stored if success and NULL otherwice
 */
char* DPDKAdapter::getDevName(uint8_t devId, char* name)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        sprintf(name, "Unknown device");
        return name;
    }
    
    EthDevInfo_t info;
    if(!getDevInfo(devId, &info))
    {
        sprintf(name, "Unknown device");
        return name;
    }

    if(info.pci_dev == NULL)
    {
        sprintf(name, "Unknown device");
        return name;
    }
    
    snprintf(name, sizeof(name), "dpdk%d", devId);

    return name;
}

/**
 * @brief           Set device default configuration
 * 
 * @param devId     uint8_t, ID of DPDK device
 * @param rxQueues  uint16_t, number of RX queues
 * @param txQueues  uint16_t, number of TX queues
 *
 * @return          0 if success and DPDK error code otherwice
 */
int DPDKAdapter::configureDev(uint8_t devId, uint16_t rxQueues, uint16_t txQueues)
{
    rte_eth_conf portConf;
    memset(&portConf, 0, sizeof(portConf));

    portConf.txmode.mq_mode = ETH_MQ_TX_NONE;

    portConf.rxmode.jumbo_frame = 1;
    portConf.rxmode.max_rx_pkt_len = MAX_PACKET_SIZE;

    return rte_eth_dev_configure(devId, rxQueues, txQueues, &portConf);
}

/**
 * @brief           Set device default configuration
 * 
 * @param devId     uint8_t, ID of DPDK device
 * @param macAddr   DevMacAddr_t*, pointer to buffer where MAC address will be stored
 *
 * @return          DevMacAddr_t* pointer to buffer where MAC address is stored and NULL otherwice
 */
DevMacAddr_t* DPDKAdapter::getDevMACAddress(uint8_t devId, DevMacAddr_t* macAddr)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return NULL;
    }

    rte_eth_macaddr_get(devId, macAddr);

    return macAddr;
}

/**
 * @brief           Initialize device RX queue
 * 
 * @param devId     uint8_t, ID of DPDK device
 * @param queueId   uint16_t, RX queue ID to initialize
 * @param desc      uint16_t, max queue description length
 *
 * @return          0 if success and DPDK related error code otherwice
 */
int DPDKAdapter::initRxQueue(uint8_t devId, uint16_t queueId, uint16_t desc)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return -1;
    }

    qDebug("Initializing RX queue %d of device %d", queueId, devId);

    struct rte_eth_rxconf rxConf;

    memset(&rxConf, 0, sizeof(rte_eth_rxconf));

    //Set default RX queue params
    rxConf.rx_free_thresh = DPDK_RX_FREE_THRESH;
    rxConf.rx_thresh.pthresh = DPDK_RX_PTHRESH;
    rxConf.rx_thresh.hthresh = DPDK_RX_HTHRESH;
    rxConf.rx_thresh.wthresh = DPDK_RX_WTHRESH;

    MemPool_t* mp = findMPool(devId, RX_);
    
    int ret = rte_eth_rx_queue_setup(devId, 
                                     queueId, 
                                     desc,
                                     SOCKET_ID_ANY,
                                     &rxConf,
                                     mp);

    if(ret < 0)
    {
        qCritical("RX Queue setup error: err=%d, dev=%u", ret, devId);
    }

    return ret;
}

/**
 * @brief           Initialize device TX queue
 * 
 * @param devId     uint8_t, ID of DPDK device
 * @param queueId   uint16_t, TX queue ID to initialize
 * @param desc      uint16_t, max queue description length
 *
 * @return          0 if success and DPDK related error code otherwice
 */
int DPDKAdapter::initTxQueue(uint8_t devId, uint16_t queueId, uint16_t desc)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return -1;
    }

    qDebug("Initializing TX queue %d of device %d", queueId, devId);

    struct rte_eth_txconf txConf;
    
    //Set default TX queue params
    memset(&txConf, 0, sizeof(rte_eth_txconf));
    txConf.tx_thresh.pthresh = DPDK_TX_PTHRESH;
    txConf.tx_thresh.hthresh = DPDK_TX_HTHRESH;
    txConf.tx_thresh.wthresh = DPDK_TX_WTHRESH;

    int ret = rte_eth_tx_queue_setup(devId,
                                     queueId, 
                                     desc,
                                     SOCKET_ID_ANY,
                                     &txConf);

    if(ret < 0)
        qCritical("TX Queue setup error: err=%d, dev=%u", ret, (unsigned) devId);
    
    return ret;
}

/**
 * @brief           Start TX
 */
void DPDKAdapter::startTx()
{
    txGlobalStart = true;
    rte_mb();
}

/**
 * @brief           Stop TX
 */
void DPDKAdapter::stopTx()
{
    txGlobalStart = false;
    rte_mb();
}

/**
 * @brief           Check if TX is started
 *
 * @return          true if started
 */
bool DPDKAdapter::isTxStarted()
{
    rte_mb();
    return txGlobalStart;
}

/**
 * @brief           Start TX on device
 *
 * @param devId     Port number
 *
 * @return          true on success
 */
bool DPDKAdapter::startTx(uint8_t devId)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return false;
    }

    qDebug("devId %u", devId);

    DeviceInfo& devInfo = devices[devId];
    devInfo.txDevStart = true;
    rte_mb();

    return true;
}

/**
 * @brief           Stop TX on device
 *
 * @param devId     Port number
 *
 * @return          true on success
 */
bool DPDKAdapter::stopTx(uint8_t devId)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return false;
    }

    qDebug("devId %u", devId);

    DeviceInfo& devInfo = devices[devId];
    devInfo.iterationCnt = 0;
    devInfo.txDevStart = false;
    rte_mb();

    return true;
}

/**
 * @brief           Check if TX is started on device
 *
 * @param devId     Port number
 *
 * @return          true if started
 */
bool DPDKAdapter::isTxStarted(uint8_t devId)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return false;
    }

    rte_mb();
    DeviceInfo& devInfo = devices[devId];
    return devInfo.txDevStart;
}

/**
 * @brief           Start RX
 */
void DPDKAdapter::startRx()
{
    rxGlobalStart = true;
    rte_mb();
}

/**
 * @brief           Stop RX
 */
void DPDKAdapter::stopRx()
{
    rxGlobalStart = false;
    rte_mb();
}

/**
 * @brief           Check if RX is started
 *
 * @return          true if started
 */
bool DPDKAdapter::isRxStarted()
{
    rte_mb();
    return rxGlobalStart;
}

/**
 * @brief                    Initiate a capture
 *
 * @param devId              Port number
 * @param captureData        A pointer to the capture buffer
 * @param captureDataLength  Size of a capture buffer
 *
 * @return                   true on success
 */
bool DPDKAdapter::startRx(uint8_t devId, char *captureData, unsigned int captureDataLength)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return false;
    }

    qDebug("devId %u, allocated a capture buffer of size %u bytes ", devId, captureDataLength);

    memset(captureData, 0, captureDataLength);

    DeviceInfo& devInfo = devices[devId];

    devInfo.captureDataLength = captureDataLength;
    devInfo.captureDataSize = 0;
    devInfo.captureData = captureData;

    // store the number of used descriptors in RX ring
    devInfo.rxQueueCount = rte_eth_rx_queue_count(devId, 0);
    qDebug("RX queue 0 count %d\n", devInfo.rxQueueCount);

    devInfo.rxDevStart = true;

    devInfo.rxTicksStart = rte_get_tsc_cycles();

    rte_mb();

    return true;
}

/**
 * @brief                   Stop a capture
 *
 * @param devId             Port number
 * @param pCaptureDataSize  Number of captured bytes
 *
 * @return                  true on success
 */
bool DPDKAdapter::stopRx(uint8_t devId, unsigned int *pCaptureDataSize)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return false;
    }

    qDebug("devId %u", devId);

    DeviceInfo& devInfo = devices[devId];
    devInfo.rxDevStart = false;
    *pCaptureDataSize = devInfo.captureDataSize;

    rte_mb();

    qDebug("Captured %u bytes", devInfo.captureDataSize);

    return true;
}

/**
 * @brief           Check if capture is started on the port
 *
 * @param   devId   Port number
 *
 * @return          true if started
 */
bool DPDKAdapter::isRxStarted(uint8_t devId)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return false;
    }

    rte_mb();
    DeviceInfo& devInfo = devices[devId];
    return devInfo.rxDevStart;
}

/**
 * @brief           Start DPDK device
 * 
 * @param devId     uint8_t, ID of DPDK device
 *
 * @return          0 if success and DPDK related error code otherwice
 */
int DPDKAdapter::startDev(uint8_t devId)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return -1;
    }

    int ret = rte_eth_dev_start(devId);

    if(ret < 0)
        qCritical("Can not start device: err=%d, dev=%u", ret, (unsigned) devId);

    return ret;
}

/**
 * @brief           Stop DPDK device
 * 
 * @param devId     uint8_t, ID of DPDK device
 */
void DPDKAdapter::stopDev(uint8_t devId)
{
	if(devId > RTE_MAX_ETHPORTS)
	{
		qCritical("Device ID is out of range");
		return;
	}

	rte_eth_dev_stop(devId);
	
}

/**
 * @brief           Get device link state
 * 
 * @param devId     uint8_t, ID of DPDK device
 * @param link      EthLinkState_t*, pointer to buffer where device link state will be stored
 * 
 * @return          true if success and false otherwice
 */
bool DPDKAdapter::getDevLinkState(uint8_t devId, EthLinkState_t* link)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return false;
    }
    
    memset(link, 0, sizeof(EthLinkState_t));

    rte_eth_link_get_nowait(devId, link);

    return true;
}

/**
 * @brief           Set device promiscuous mode
 * 
 * @param devId     uint8_t, ID of DPDK device
 * @param promisc   bool, true to enable or false disable promiscuous mode
 * 
 * @return          true if success and false otherwice
 */
bool DPDKAdapter::setDevPromisc(uint8_t devId, bool promisc)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return false;
    }

    if(promisc)
        rte_eth_promiscuous_enable(devId);
    else
        rte_eth_promiscuous_disable(devId);

    return true;
}

/**
 * @brief           Get device promiscuous mode
 * 
 * @param devId     uint8_t, ID of DPDK device
 * @param promisc   bool*, pointer to buffer where retrieved promiscuous mode will be stored
 * 
 * @return          true if success and false otherwice
 */
bool DPDKAdapter::getDevPromisc(uint8_t devId, bool* promisc)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return false;
    }

    *promisc = (1 == rte_eth_promiscuous_get(devId));

    return true;
}

/**
 * @brief           Reset device statistics data on HW
 * 
 * @param devId     uint8_t, ID of DPDK device
 */
void DPDKAdapter::resetDevStats(uint8_t devId)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return;
    }

    rte_eth_stats_reset(devId);
}

/**
 * @brief           Retrieve device statistics data from HW
 * 
 * @param devId     uint8_t, ID of DPDK device
 * @param stat      EthDevStatistics_t*, pointer to buffer where retrieved data will be stored
 */
void DPDKAdapter::getDevStatistics(uint8_t devId, EthDevStatistics_t* stat)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return;
    }

    rte_eth_stats_get(devId, stat);
}

/**
 * @brief           Main routine for lcore
 * 
 * @param arg       void*, Pointer to DPDK adapter object
 * 
 * @return          routine exit error code
 */
int DPDKAdapter::lcoreMainRoutine(void* arg)
{
    unsigned int lcore_id = rte_lcore_id();

    qDebug("entering main loop on lcore %u", lcore_id);

    DPDKAdapter* pAdapter = (DPDKAdapter*)arg;

    while (pAdapter->isTxStarted())
    {
        pAdapter->txRoutine();
        pAdapter->rxRoutine();
    }
    
    return 0;
}

/**
 * @brief           TX routine
 */
void DPDKAdapter::txRoutine()
{
    uint8_t devId = 0;
    uint8_t lcoreId = rte_lcore_id();
    LcoreInfo& coreInfo = cores[lcoreId];

    for(PortList_t::iterator itor = coreInfo.txPortList.begin(); itor != coreInfo.txPortList.end(); itor++)
    {
        devId = *itor;

        if(!isTxStarted(devId))
        {
            continue;
        }

        struct rte_eth_dev *dev = &rte_eth_devices[devId];
        if(!dev || !dev->data->dev_started)
        {
            continue;
        }

        DeviceInfo& devInfo = devices[devId];

        if(devInfo.mode == DPDK_SEQ_MODE)
        {
            if((devInfo.loopMode == false) && devInfo.iterationCnt > 0)
            {
                continue;
            }
        }

        if(devInfo.mode == DPDK_INTER_MODE)
        {
            for(StreamList_t::iterator it = devInfo.streams.begin(); it != devInfo.streams.end(); it++)
            {
                StreamInfo& streamInfo = *it;

                if(streamInfo.isReadyTransmit())
                {
                    streamInfo.sendBurst(devId);
                }
            }
        }

        if(devInfo.mode == DPDK_SEQ_MODE)
        {
            StreamInfo& streamInfo = devInfo.streams[devInfo.curStream];

            if(streamInfo.isReadyTransmit())
            {
                if (streamInfo.allBurstsTransmitted())
                {
                    devInfo.curStream = devInfo.curStream < devInfo.streams.size() - 1 ? devInfo.curStream + 1 : 0;
                    streamInfo.resetBurstCnt();
                    streamInfo.resetTransmitTimer();
                    continue;
                }

                streamInfo.sendBurst(devId);
            }
         }

        devInfo.iterationCnt++;
    }
}

/**
 * @brief           Assign device ID to its object
 *
 * @param devId     uint8_t, ID of DPDK device
 *
 * @return          MBuf_t*, pointer to allocated MBuf or NULL if failed
 */
MBuf_t* DPDKAdapter::txMbufAlloc(uint8_t devId)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return NULL;
    }

    return rte_pktmbuf_alloc(findMPool(devId, TX_));
}

/**
 * @brief           Free allocated MBuf
 * 
 * @param m         MBuf_t*, pointer to allocated MBUf
 */
void DPDKAdapter::txMbufFree(MBuf_t* m)
{
    rte_pktmbuf_free(m);
}

/**
 * @brief           Send packet from specified device
 *
 * @param devId     uint8_t, ID of DPDK device
 * @param burst     MBuf_t**, pointer to packets array to be transmitted
 * @param pkt       MBuf_t**, pointer to packets array to be transmitted
 *
 * @return          int, count of transmitted packets
 */
int DPDKAdapter::sendMbufBurst(uint8_t devId, MBuf_t** burst, uint8_t size)
{
    if(devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return -1;
    }

    if(size > DPDK_TX_MAX_PKT_BURST)
    {
        qCritical("Maximum packet burst value is exceeded");
        return -2;
    }

    // Nothing to do
    if(!burst)
    {
        qWarning("There is nothing to send");
        return 0;
    }

    return rte_eth_tx_burst(devId, 0, burst, size);
}

/**
 * @brief           Send packet from specified device
 *
 * @param devId     uint8_t, ID of DPDK device
 * @param mBuf      MBuf_t*, memory buffer
 *
 * @return          int, count of transmitted packets
 */
int DPDKAdapter::sendMbufBurstWithoutFree(uint8_t devId, MBuf_t** burst, uint8_t size)
{
    if (devId > RTE_MAX_ETHPORTS)
    {
        qCritical("Device ID is out of range");
        return -1;
    }

    if (size > DPDK_TX_MAX_PKT_BURST)
    {
        qCritical("Maximum packet burst value is exceeded");
        return -2;
    }

    // Nothing to do
    if (!burst)
    {
        qWarning("There is nothing to send");
        return 0;
    }

    for (uint8_t i = 0; i < size; i++)
    {
        rte_pktmbuf_refcnt_update(burst[i], 1);
    }

    return rte_eth_tx_burst(devId, 0, burst, size);
}

/**
 * @brief            Add a basic stream
 *
 * @param devId      ID of DPDK device
 * @param delay      delay between frames
 * @param numPackets number of packets to be sent
 * @param pStreamId  stream id assigned
 *
 * @return          true on success
 */
bool DPDKAdapter::addPacketStream(uint8_t devId, unsigned int delay, unsigned int numPackets, unsigned int* pStreamId)
{
    return addBurstStream(devId, delay, 1, numPackets, pStreamId);
}

/**
 * @brief            Add a stream consisting of bursts
 *
 * @param devId      ID of DPDK device
 * @param burstDelay burst delay
 * @param burstSize  burst size
 * @param numBursts  number of bursts
 * @param pStreamId  stream id assigned
 *
 * @return          true on success
 */
bool DPDKAdapter::addBurstStream(uint8_t devId, unsigned int burstDelay, unsigned int burstSize, unsigned int numBursts, unsigned int* pStreamId)
{
    uint8_t txBurstSize = 0;

    qDebug("devId %u, burstDelay %u, burstSize %u, numBursts %u", devId, burstDelay, burstSize, numBursts);

    struct rte_eth_dev *dev = &rte_eth_devices[devId];
    if(!dev || !dev->data->dev_started)
    {
        return false;
    }

    DeviceInfo& devInfo = devices[devId];

    if (burstDelay < DPDK_TX_DELAY_THRESH)
    {
        txBurstSize = DPDK_TX_MAX_PKT_BURST;
        burstDelay = burstDelay * DPDK_TX_MAX_PKT_BURST;
    }
    else
    {
        txBurstSize = 1;
    }

    qDebug("devId %u, burstDelay %u, txBurstSize %u", devId, burstDelay, txBurstSize);

    StreamInfo stream(devId, burstDelay, burstSize, numBursts, txBurstSize);

    *pStreamId = devInfo.streams.size();

    devInfo.streams.push_back(stream);

    return true;
}

/**
 * @brief            Add a packet to a stream
 *
 * @param devId      ID of DPDK device
 * @param streamId   stream id assigned
 * @param data       pointer to buffer with data
 * @param dataLen    data buffer size
 *
 * @return          true on success
 */
bool DPDKAdapter::addPacket(uint8_t devId, unsigned int streamId, const void* data, unsigned int dataLen)
{
    qDebug("devId %u, streamId %u, dataLen %u", devId, streamId, dataLen);

    struct rte_eth_dev *dev = &rte_eth_devices[devId];
    if(!dev || !dev->data->dev_started)
    {
        return false;
    }

    DeviceInfo& devInfo = devices[devId];
    StreamInfo& stream = devInfo.streams[streamId];
    stream.addPacket(data, dataLen);

    return true;
}

/**
 * @brief           Clear the packet list
 *
 * @param devId     uint8_t, ID of DPDK device
 *
 * @return          true on success
 */
bool DPDKAdapter::clearPacketList(uint8_t devId)
{
    qDebug("devId %u", devId);

    DeviceInfo& devInfo = devices[devId];

    devInfo.streams.clear();

    devInfo.loopMode = false;
    devInfo.mode = DPDK_SEQ_MODE;
    devInfo.iterationCnt = 0;

    return true;
}

/**
 * @brief           Set packet list loop mode
 *
 * @param devId     ID of DPDK device
 * @param loop      True if a continues transmit
 *
 */
void DPDKAdapter::setPacketListLoopMode(uint8_t devId, bool loop)
{
    qDebug("devId %u, loop %u", devId, loop);

    DeviceInfo& devInfo = devices[devId];
    devInfo.loopMode = loop;
}

/**
 * @brief           Set packet list mix mode
 *
 * @param devId     ID of DPDK device
 * @param mode      transmission mode
 *
 */
void DPDKAdapter::setPacketListMixMode(uint8_t devId, dpdk_tx_mode_t mode)
{
    qDebug("devId %u, mode %u", devId, mode);

    DeviceInfo& devInfo = devices[devId];
    devInfo.mode = mode;
}

/**
 * @brief           Copy all mbuf segments to a buffer
 *
 * @param devId     port number
 * @param pMbuf     mbuf
 * @param data      Data buffer
 * @param dataLen   Data buffer length
 *
 * @return          true on success
 */
bool DPDKAdapter::copyMbufToBuf(uint8_t devId, MBuf_t* pMbuf, char* data, unsigned int& dataLen)
{
    qDebug("pkt_len %u, data_len %u, nb_segs %u", pMbuf->pkt.pkt_len, pMbuf->pkt.data_len, pMbuf->pkt.nb_segs);

    unsigned int segCnt = pMbuf->pkt.nb_segs;
    unsigned int offset = 0;

    MBuf_t* pNextMbuf = pMbuf;
    dataLen = pMbuf->pkt.pkt_len;

    while (segCnt > 0)
    {
        MBuf_t* pCurMbuf = pNextMbuf;
        qDebug("segCnt %u, offset %u", segCnt, offset);

        rte_memcpy(data + offset, pCurMbuf->pkt.data, pCurMbuf->pkt.data_len);

        qDebug("pkt_len %u, data_len %u", pCurMbuf->pkt.pkt_len, pCurMbuf->pkt.data_len);

        offset += pCurMbuf->pkt.data_len;
        pNextMbuf = pCurMbuf->pkt.next;

        rte_pktmbuf_free(pCurMbuf);

        segCnt--;
    }

    return true;
}

/**
 * @brief           RX routine
 */
void DPDKAdapter::rxRoutine()
{
    uint8_t pkt = 0;
    uint8_t rxPktCount = 0;
    uint8_t devId = 0;

    uint8_t lcoreId = rte_lcore_id();
    LcoreInfo& coreInfo = cores[lcoreId];

    for(PortList_t::iterator itor = coreInfo.rxPortList.begin(); itor != coreInfo.rxPortList.end(); itor++)
    {
        devId = *itor;

        DeviceInfo& devInfo = devices[devId];

        struct rte_eth_dev *dev = &rte_eth_devices[devId];
        if(!dev || !dev->data->dev_started)
        {
            continue;
        }

        rxPktCount = rte_eth_rx_burst(devId, 0, devInfo.rxBurstBuf, DPDK_RX_MAX_PKT_BURST);

        if(isRxStarted(devId))
        {
            saveToBuf(devId, devInfo.rxBurstBuf, rxPktCount);
        }

        for(pkt = 0; pkt < rxPktCount; pkt++)
        {
            rte_pktmbuf_free(devInfo.rxBurstBuf[pkt]);
        }
    }
}

/**
 * @brief           Copy a buffer to mbuf
 *
 * @param devId     port number
 * @param data      Data buffer
 * @param dataLen   Data buffer length
 * @param pMbuf     mbuf
 *
 * @return          true on success
 */
bool DPDKAdapter::copyBufToMbuf(uint8_t devId, char* data, unsigned int dataLen, MBuf_t*& pMbuf)
{
    unsigned int offset = 0;
    pMbuf = NULL;

    pMbuf = DPDKAdapter::instance()->txMbufAlloc(devId);
    if (pMbuf == NULL)
    {
        qCritical("No mbuf available");
        return false;
    }

    pMbuf->pkt.data_len = dataLen < MAX_SEG_SIZE ? dataLen : MAX_SEG_SIZE;
    pMbuf->pkt.pkt_len = dataLen;
    pMbuf->pkt.nb_segs = (dataLen / MAX_SEG_SIZE) + ((dataLen % MAX_SEG_SIZE) || 0);

    qDebug("pkt_len %u, data_len %u, nb_segs %u",pMbuf->pkt.pkt_len, pMbuf->pkt.data_len, pMbuf->pkt.nb_segs);

    rte_memcpy(pMbuf->pkt.data, data, pMbuf->pkt.data_len);

    if (dataLen <= MAX_SEG_SIZE)
    {
        return true;
    }

    dataLen -= pMbuf->pkt.data_len;
    offset = pMbuf->pkt.data_len;

    MBuf_t* pCurMbuf = pMbuf;

    while (dataLen > 0)
    {
        qDebug("offset %u, dataLen %u", offset, dataLen);

        pCurMbuf->pkt.next = DPDKAdapter::instance()->txMbufAlloc(devId);
        if (pCurMbuf->pkt.next == NULL)
        {
            qCritical("No mbuf available");
            return false;
        }

        pCurMbuf = pCurMbuf->pkt.next;

        pCurMbuf->pkt.data_len = dataLen < MAX_SEG_SIZE ? dataLen : MAX_SEG_SIZE;

        qDebug("pkt_len %u, data_len %u, nb_segs %u",pCurMbuf->pkt.pkt_len, pCurMbuf->pkt.data_len, pCurMbuf->pkt.nb_segs);

        rte_memcpy(pCurMbuf->pkt.data, data + offset, pCurMbuf->pkt.data_len);

        dataLen -= pCurMbuf->pkt.data_len;
        offset += pCurMbuf->pkt.data_len;
    }

    return true;
}

/**
 * @brief           Clone mbuf
 *
 * @param devId     Port number
 * @param pMbufIn   mbuf
 *
 * @return          true on success
 */
MBuf_t* DPDKAdapter::cloneMbuf(uint8_t devId, const MBuf_t* pMbufIn)
{
    MBuf_t* pMbufOut = NULL;

    if (pMbufIn == NULL)
    {
        qCritical("No mbuf provided");
        return NULL;
    }

    pMbufOut = DPDKAdapter::instance()->txMbufAlloc(devId);
    if (pMbufOut == NULL)
    {
        qCritical("No mbuf available");
        return NULL;
    }

    rte_memcpy(pMbufOut->pkt.data, pMbufIn->pkt.data, pMbufIn->pkt.data_len);
    pMbufOut->pkt.nb_segs = pMbufIn->pkt.nb_segs;
    pMbufOut->pkt.data_len = pMbufIn->pkt.data_len;
    pMbufOut->pkt.pkt_len = pMbufIn->pkt.pkt_len;

    MBuf_t* pCurMbufOut = pMbufOut;
    MBuf_t* pCurMbufIn = pMbufIn->pkt.next;

    while (pCurMbufIn != 0)
    {
        pCurMbufOut->pkt.next = DPDKAdapter::instance()->txMbufAlloc(devId);
        if (pCurMbufOut == NULL)
        {
            qCritical("No mbuf available");
            return NULL;
        }

        pCurMbufOut = pCurMbufOut->pkt.next;

        qDebug("pkt_len %u, data_len %u, nb_segs %u", pCurMbufIn->pkt.pkt_len, pCurMbufIn->pkt.data_len, pCurMbufIn->pkt.nb_segs);

        rte_memcpy(pCurMbufOut->pkt.data, pCurMbufIn->pkt.data, pCurMbufIn->pkt.data_len);
        pCurMbufOut->pkt.nb_segs = pCurMbufIn->pkt.nb_segs;
        pCurMbufOut->pkt.data_len = pCurMbufIn->pkt.data_len;
        pCurMbufOut->pkt.pkt_len = pCurMbufIn->pkt.pkt_len;

        pCurMbufIn = pCurMbufIn->pkt.next;
    }

    return pMbufOut;
}

/**
 * @brief           Save mbuf burst to the capture buffer
 *
 * @param devId     Port number
 * @param burstBuf  mbuf burst
 * @param pktCount  Number of packets in the burst
 *
 * @return          true on success
 */
void DPDKAdapter::saveToBuf(uint8_t devId, MBuf_t** burstBuf, uint8_t pktCount)
{
    MBuf_t* m = NULL;
    DeviceInfo& devInfo = devices[devId];

    uint64_t rxTicksEnd = rte_get_tsc_cycles();
    uint64_t ticksDiff = rxTicksEnd - devInfo.rxTicksStart;
    uint64_t timestamp = (SEC_TO_NSEC * ticksDiff) / rte_get_tsc_hz();

    struct pcap_pkthdr hdr;
    memset(&hdr, 0, sizeof(pcap_pkthdr));

    uint32_t sec = timestamp / SEC_TO_NSEC;
    hdr.ts.tv_sec = sec;
    uint32_t usec = (timestamp - hdr.ts.tv_sec * SEC_TO_NSEC) / MSEC_TO_NSEC;
    hdr.ts.tv_usec = usec;

    for(uint8_t pkt = 0; pkt < pktCount; pkt++)
    {
        m = burstBuf[pkt];

        hdr.caplen = m->pkt.data_len;
        hdr.len = hdr.caplen;

        if(devInfo.captureDataSize + sizeof(hdr) + m->pkt.data_len > devInfo.captureDataLength)
        {
            qDebug("Capture buffer is full with %u bytes", devInfo.captureDataSize);
            devInfo.captureDataSize = 0;
        }

        memcpy(devInfo.captureData + devInfo.captureDataSize, &hdr, sizeof(hdr));
        devInfo.captureDataSize += sizeof(hdr);

        memcpy(devInfo.captureData + devInfo.captureDataSize, m->pkt.data, m->pkt.data_len);
        devInfo.captureDataSize += m->pkt.data_len;
    }
}

