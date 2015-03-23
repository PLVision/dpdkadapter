/*-
 * @file dpdk_adapter.h
 *
 * The file contains classes declaration to work with DPDK library.
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

#ifndef DPDKADAPTER_H
#define DPDKADAPTER_H

#include "stdint.h"

#include "rte_config.h"
#include "rte_ethdev.h"

#include "dpdk_api.h"

#include <vector>
#include <set>
#include <map>
#include <string>

typedef struct rte_mbuf             MBuf_t;
typedef struct rte_mempool          MemPool_t;
typedef struct rte_eth_dev_info     EthDevInfo_t;
typedef struct ether_addr           DevMacAddr_t;
typedef struct rte_eth_rxconf       EthRxConf_t;
typedef struct rte_eth_txconf       EthTxConf_t;
typedef struct rte_eth_link         EthLinkState_t;
typedef struct rte_eth_stats        EthDevStatistics_t;

class DPDKProfiler
{
public:
    DPDKProfiler(uint8_t coreId, const char* name);
    ~DPDKProfiler();

    static uint64_t lastDurationGet(uint8_t coreId, std::string& name);
    static uint64_t medDurationGet(uint8_t coreId, std::string& name);
    static uint64_t invokeCountGet(uint8_t coreId, std::string& name);

    static void reset(uint8_t coreId, std::string& name);

private:
    struct Stats_t
    {
        uint64_t last_duration;
        uint64_t total_duration;
        uint64_t invoke_cnt;
    };

    typedef std::map<std::string, Stats_t> StatsMap_t;

    uint64_t start_;
    static StatsMap_t stats_[RTE_MAX_LCORE];

    uint8_t coreId_;
    std::string name_;
};

class DPDKAdapter
{
public:
    static const uint16_t DPDK_MEMPOOL_SIZE = 640;
    static const uint8_t DPDK_MEMPOOL_CACHE_SIZE = 32;

    static const uint16_t DPDK_RX_DESC_NUMBER = 512;
    static const uint16_t DPDK_TX_DESC_NUMBER = 512;

    static const uint8_t DPDK_RX_MAX_PKT_BURST = 128;
    static const uint8_t DPDK_TX_MAX_PKT_BURST = 32;

    static const uint8_t DPDK_RX_FREE_THRESH = 0;
    static const uint8_t DPDK_RX_PTHRESH = 8;
    static const uint8_t DPDK_RX_HTHRESH = 8;
    static const uint8_t DPDK_RX_WTHRESH = 0;

    static const uint8_t DPDK_TX_PTHRESH = 32;
    static const uint8_t DPDK_TX_HTHRESH = 0;
    static const uint8_t DPDK_TX_WTHRESH = 0;

    static const uint32_t DPDK_TX_DELAY_THRESH = 1000000;

    class PacketInfo
    {
    public:
        PacketInfo(uint8_t devId, const void* data, unsigned int dataLen);
        PacketInfo(const PacketInfo& other);
        ~PacketInfo();
        MBuf_t* getMbuf();

    public:
        char* data_;
        unsigned int dataLen_;

    private:
        MBuf_t*             mbuf_;
        uint8_t             devId_;
    };

    class StreamInfo
    {
    private:
        StreamInfo();

    public:
        StreamInfo(unsigned char devId, unsigned int delay, unsigned int burstSize, unsigned int numBursts, uint8_t txBurstSize);
        StreamInfo(const StreamInfo& other);
        ~StreamInfo();
        bool isReadyTransmit();
        void resetTransmitTimer();
        bool allBurstsTransmitted();
        void nPause();
        bool sendBurst(uint8_t devId);
        void resetPktCnt();
        void resetBurstCnt();
        void addPacket(const void* data, unsigned int dataLen);
        unsigned int getNextPacketId(unsigned int curPacketId);
        bool getMbuf(unsigned int packetId, MBuf_t*& pMBuf);
        bool getPacket(unsigned int packetId, PacketInfo*& pPacket);

    private:
        uint32_t            delay_;
        uint64_t            ticksDelay_;
        uint64_t            lastTx_;
        uint32_t            numBursts_;
        uint32_t            burstSize_;
        uint32_t            sentPackets_;
        uint32_t            sentBursts_;
        uint8_t             devId_;
        uint8_t             txBurstSize_;

        typedef std::vector<PacketInfo> PacketList_t;
        PacketList_t        packets_;

    public:
        uint32_t            curPacketId_;
    };

    typedef std::vector<StreamInfo> StreamList_t;
    typedef std::set<uint8_t> PortList_t;

    struct LcoreInfo
    {
        PortList_t rxPortList;
        PortList_t txPortList;
    };

    struct DeviceInfo
    {
        DeviceInfo();
        ~DeviceInfo();

        bool                        txDevStart;
        bool                        rxDevStart;
        uint64_t                    rxTicksStart;
        bool                        loopMode;
        uint32_t                    iterationCnt;
        StreamList_t                streams;
        uint32_t                    curStream;
        dpdk_tx_mode_t              mode;
        uint32_t                    rxQueueCount;

        char*                       rxData;
        MBuf_t*                     txBurstBuf[DPDK_TX_MAX_PKT_BURST];
        MBuf_t*                     rxBurstBuf[DPDK_RX_MAX_PKT_BURST];

        char                        *captureData;
        uint32_t                    captureDataLength;
        uint32_t                    captureDataSize;
    };

private:
    // This is singleton
    DPDKAdapter();
    static DPDKAdapter* instance_;

public:
    static DPDKAdapter* instance();
    static void     destroy();
    virtual         ~DPDKAdapter();
    bool            init(int& argc, char**& argv);
    bool            initPort(uint8_t devId);
    uint8_t         getPortsCount() const { return nPortCount; }
    char*           getDevName(uint8_t devId, char* name);
    DevMacAddr_t*   getDevMACAddress(uint8_t devId, DevMacAddr_t* macAddr);

    bool            getDevLinkState(uint8_t devId, EthLinkState_t* link);
    void            getDevStatistics(uint8_t devId, EthDevStatistics_t* stat);
    void            resetDevStats(uint8_t devId);

    bool            getDevPromisc(uint8_t devId, bool* promisc);
    bool            setDevPromisc(uint8_t devId, bool promisc);

    int             startDev(uint8_t devId);
    void            stopDev(uint8_t devId);

    bool            addPacketStream(uint8_t devId, unsigned int delay, unsigned int numPackets, unsigned int* pStreamId);
    bool            addBurstStream(uint8_t devId, unsigned int burstDelay, unsigned int burstSize, unsigned int numBursts, unsigned int* pStreamId);
    bool            addPacket(uint8_t devId, unsigned int streamId, const void* data, unsigned int dataLen);
    bool            clearPacketList(uint8_t devId);
    void            setPacketListLoopMode(uint8_t devId, bool loop);
    void            setPacketListMixMode(uint8_t devId, dpdk_tx_mode_t mode);

    bool            startTx(uint8_t devId);
    bool            stopTx(uint8_t devId);
    bool            isTxStarted(uint8_t devId);

    bool            startRx(uint8_t devId, char *captureData, unsigned int captureDataLength);
    bool            stopRx(uint8_t devId, unsigned int *pCaptureDataSize);
    bool            isRxStarted(uint8_t devId);
    
private:
    MemPool_t*      rxMemoryPool;
    
    uint8_t         nPortCount;
    LcoreInfo       cores[RTE_MAX_LCORE];
    DeviceInfo      devices[RTE_MAX_ETHPORTS];

    bool            txGlobalStart;
    bool            rxGlobalStart;
    bool            initialized;    

private:
    bool            isInitialized();
    bool            initializeDevs();
    bool            portRxCoreMap(uint8_t devId, uint8_t coreId);
    bool            portTxCoreMap(uint8_t devId, uint8_t coreId);

    EthDevInfo_t*   getDevInfo(uint8_t devId, EthDevInfo_t* info);
    int             configureDev(uint8_t devId, uint16_t nRxQueues, uint16_t nTxQueues);

    void            startTx();
    void            stopTx();
    bool            isTxStarted();

    void            startRx();
    void            stopRx();
    bool            isRxStarted();

    bool            setDevTxMode(uint8_t devId, bool mode);
    bool            setDevRxMode(uint8_t devId, bool mode);
    
    void            mbufPoolNameBuilder(unsigned int devId, char* name, int nameLen, const char* prefix);
    
    bool            initDevRxTxMPool(unsigned int devId);
    bool            initDevMBufPool(const char* name);
    MemPool_t*      findMPool(uint8_t devId, const char* prefix);
    
    int             initRxQueue(uint8_t devId, uint16_t rxQueueId, uint16_t rxDesc);
    int             initTxQueue(uint8_t devId, uint16_t txQueueId, uint16_t txDesc);

    static int      lcoreMainRoutine(void* arg);
    bool            copyBufToMbuf(uint8_t devId, char* data, unsigned int dataLen, MBuf_t*& pMbuf);
    bool            copyMbufToBuf(uint8_t devId, MBuf_t* pMbuf, char* data, unsigned int& dataLen);
    MBuf_t*         cloneMbuf(uint8_t devId, const MBuf_t* pMbufIn);
    int             sendMbufBurst(uint8_t devId, MBuf_t** burst, uint8_t size);
    int             sendMbufBurstWithoutFree(uint8_t devId, MBuf_t** burst, uint8_t size);

    MBuf_t*         txMbufAlloc(uint8_t devId);
    void            txMbufFree(MBuf_t* m);

    void            txRoutine();
    void            rxRoutine();

    void            saveToBuf(uint8_t devId, MBuf_t** burstBuf, uint8_t pktCount);
};

#endif // DPDKADAPTER_H
