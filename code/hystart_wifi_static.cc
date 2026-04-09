/*
 * HyStart++ (RFC 9406) — Wireless 802.11 Static Topology
 *
 *   - Vary nodes      : 20, 40, 60, 80, 100
 *   - Vary flows      : 10, 20, 30, 40, 50
 *   - Vary pkt/s      : 100, 200, 300, 400, 500
 *   - Vary coverage   : Tx_range × {1, 2, 3, 4, 5}
 *   - Metrics         : throughput, delay, PDR, drop ratio, energy
 *
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HyStartPlusPlusWifiStatic");

class TcpHyStartPlusPlus : public TcpLinuxReno
{
  public:
    enum HyStartPhase
    {
        HYSTART_SS,
        HYSTART_CSS,
        HYSTART_CA
    };

    static TypeId GetTypeId();
    TcpHyStartPlusPlus();
    TcpHyStartPlusPlus(const TcpHyStartPlusPlus& sock);
    ~TcpHyStartPlusPlus() override = default;

    std::string GetName() const override;
    void IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked) override;
    void PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt) override;
    uint32_t GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight) override;
    void CongestionStateSet(Ptr<TcpSocketState> tcb,
                            const TcpSocketState::TcpCongState_t newState) override;
    Ptr<TcpCongestionOps> Fork() override;

  private:
    Time minRttThresh;
    Time maxRttThresh;
    uint32_t minRttDivisor;
    uint32_t nRttSample;
    uint32_t cssGrowthDiv;
    uint32_t cssMaxRounds;
    uint32_t m_L;

    HyStartPhase phase;
    bool roundStarted;
    SequenceNumber32 windowEnd;

    Time lastRoundMinRtt;
    Time currentRoundMinRtt;
    uint32_t rttSampleCount;

    static constexpr uint32_t RTT_WINDOW_SIZE = 3;
    std::vector<Time> m_rttWindow;

    Time m_cssBaselineMinRtt;
    uint32_t cssRoundCount;
    uint32_t m_cssCwndAccum;
    bool m_isInitialSlowStart;

    void BeginNewRttRound(Ptr<TcpSocketState> tcb);
    Time ComputeRttThresh() const;
    static const char* PhaseName(HyStartPhase p);
};

NS_OBJECT_ENSURE_REGISTERED(TcpHyStartPlusPlus);

TypeId
TcpHyStartPlusPlus::GetTypeId()
{
    static TypeId tid = TypeId("ns3::TcpHyStartPlusPlus")
                            .SetParent<TcpLinuxReno>()
                            .SetGroupName("Internet")
                            .AddConstructor<TcpHyStartPlusPlus>()
                            .AddAttribute("MinRttThresh",
                                          "MIN_RTT_THRESH",
                                          TimeValue(MilliSeconds(4)),
                                          MakeTimeAccessor(&TcpHyStartPlusPlus::minRttThresh),
                                          MakeTimeChecker())
                            .AddAttribute("MaxRttThresh",
                                          "MAX_RTT_THRESH",
                                          TimeValue(MilliSeconds(16)),
                                          MakeTimeAccessor(&TcpHyStartPlusPlus::maxRttThresh),
                                          MakeTimeChecker())
                            .AddAttribute("MinRttDivisor",
                                          "MIN_RTT_DIVISOR",
                                          UintegerValue(8),
                                          MakeUintegerAccessor(&TcpHyStartPlusPlus::minRttDivisor),
                                          MakeUintegerChecker<uint32_t>(1))
                            .AddAttribute("NRttSample",
                                          "N_RTT_SAMPLE",
                                          UintegerValue(8),
                                          MakeUintegerAccessor(&TcpHyStartPlusPlus::nRttSample),
                                          MakeUintegerChecker<uint32_t>(1))
                            .AddAttribute("CssGrowthDivisor",
                                          "CSS_GROWTH_DIVISOR",
                                          UintegerValue(4),
                                          MakeUintegerAccessor(&TcpHyStartPlusPlus::cssGrowthDiv),
                                          MakeUintegerChecker<uint32_t>(2))
                            .AddAttribute("CssMaxRounds",
                                          "CSS_ROUNDS",
                                          UintegerValue(5),
                                          MakeUintegerAccessor(&TcpHyStartPlusPlus::cssMaxRounds),
                                          MakeUintegerChecker<uint32_t>(1))
                            .AddAttribute("L",
                                          "Aggressiveness limit",
                                          UintegerValue(8),
                                          MakeUintegerAccessor(&TcpHyStartPlusPlus::m_L),
                                          MakeUintegerChecker<uint32_t>(1));
    return tid;
}

TcpHyStartPlusPlus::TcpHyStartPlusPlus()
    : TcpLinuxReno(),
      minRttThresh(MilliSeconds(4)),
      maxRttThresh(MilliSeconds(16)),
      minRttDivisor(8),
      nRttSample(8),
      cssGrowthDiv(4),
      cssMaxRounds(5),
      m_L(8),
      phase(HYSTART_SS),
      roundStarted(false),
      windowEnd(SequenceNumber32(0)),
      lastRoundMinRtt(Time::Max()),
      currentRoundMinRtt(Time::Max()),
      rttSampleCount(0),
      m_rttWindow(),
      m_cssBaselineMinRtt(Time::Max()),
      cssRoundCount(0),
      m_cssCwndAccum(0),
      m_isInitialSlowStart(true)
{
}

TcpHyStartPlusPlus::TcpHyStartPlusPlus(const TcpHyStartPlusPlus& sock)
    : TcpLinuxReno(sock),
      minRttThresh(sock.minRttThresh),
      maxRttThresh(sock.maxRttThresh),
      minRttDivisor(sock.minRttDivisor),
      nRttSample(sock.nRttSample),
      cssGrowthDiv(sock.cssGrowthDiv),
      cssMaxRounds(sock.cssMaxRounds),
      m_L(sock.m_L),
      phase(sock.phase),
      roundStarted(sock.roundStarted),
      windowEnd(sock.windowEnd),
      lastRoundMinRtt(sock.lastRoundMinRtt),
      currentRoundMinRtt(sock.currentRoundMinRtt),
      rttSampleCount(sock.rttSampleCount),
      m_rttWindow(sock.m_rttWindow),
      m_cssBaselineMinRtt(sock.m_cssBaselineMinRtt),
      cssRoundCount(sock.cssRoundCount),
      m_cssCwndAccum(sock.m_cssCwndAccum),
      m_isInitialSlowStart(sock.m_isInitialSlowStart)
{
}

const char*
TcpHyStartPlusPlus::PhaseName(HyStartPhase p)
{
    switch (p)
    {
    case HYSTART_SS:
        return "SS";
    case HYSTART_CSS:
        return "CSS";
    case HYSTART_CA:
        return "CA";
    }
    return "??";
}

std::string
TcpHyStartPlusPlus::GetName() const
{
    return "TcpHyStartPlusPlus";
}

void
TcpHyStartPlusPlus::BeginNewRttRound(Ptr<TcpSocketState> tcb)
{
    lastRoundMinRtt = currentRoundMinRtt;
    currentRoundMinRtt = Time::Max();
    rttSampleCount = 0;
    m_rttWindow.clear();
    windowEnd = tcb->m_highTxMark;
}

Time
TcpHyStartPlusPlus::ComputeRttThresh() const
{
    Time fraction = Time::FromDouble(lastRoundMinRtt.GetDouble() / (double)minRttDivisor, Time::NS);
    return std::max(minRttThresh, std::min(fraction, maxRttThresh));
}

void
TcpHyStartPlusPlus::PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt)
{
    if (rtt.IsZero() || rtt.IsNegative())
    {
        return;
    }
    if (!m_isInitialSlowStart)
    {
        return;
    }
    if (phase == HYSTART_CA)
    {
        return;
    }

    if (!roundStarted)
    {
        windowEnd = tcb->m_highTxMark;
        roundStarted = true;
    }

    // Weighted-average smoother (0.2 : 0.3 : 0.5)
    m_rttWindow.push_back(rtt);
    if (m_rttWindow.size() > RTT_WINDOW_SIZE)
    {
        m_rttWindow.erase(m_rttWindow.begin());
    }

    Time smoothedRtt;
    if (m_rttWindow.size() == RTT_WINDOW_SIZE)
    {
        smoothedRtt =
            Time::FromDouble(m_rttWindow[0].GetDouble() * 0.2 + m_rttWindow[1].GetDouble() * 0.3 +
                                 m_rttWindow[2].GetDouble() * 0.5,
                             Time::NS);
    }
    else
    {
        smoothedRtt = rtt;
    }

    currentRoundMinRtt = std::min(currentRoundMinRtt, smoothedRtt);
    rttSampleCount++;

    if (tcb->m_lastAckedSeq >= windowEnd)
    {
        if (phase == HYSTART_CSS)
        {
            cssRoundCount++;
            if (cssRoundCount >= cssMaxRounds)
            {
                phase = HYSTART_CA;
                tcb->m_ssThresh = tcb->m_cWnd;
                return;
            }
        }
        BeginNewRttRound(tcb);
    }

    if (phase == HYSTART_SS)
    {
        if (rttSampleCount >= nRttSample && currentRoundMinRtt != Time::Max() &&
            lastRoundMinRtt != Time::Max())
        {
            Time rttThresh = ComputeRttThresh();
            Time delayTarget = lastRoundMinRtt + rttThresh;

            if (currentRoundMinRtt >= delayTarget)
            {
                m_cssBaselineMinRtt = currentRoundMinRtt;
                phase = HYSTART_CSS;
                cssRoundCount = 0;
                m_cssCwndAccum = 0;
            }
        }
    }
    else if (phase == HYSTART_CSS)
    {
        if (rttSampleCount >= nRttSample && currentRoundMinRtt < m_cssBaselineMinRtt)
        {
            m_cssBaselineMinRtt = Time::Max();
            phase = HYSTART_SS;
        }
    }
}

void
TcpHyStartPlusPlus::IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
    uint32_t effectiveL = tcb->m_pacing ? UINT32_MAX : m_L;

    if (tcb->m_cWnd < tcb->m_ssThresh)
    {
        if (phase == HYSTART_SS)
        {
            uint32_t limited = std::min(segmentsAcked, effectiveL);
            tcb->m_cWnd = std::min(tcb->m_cWnd.Get() + limited * tcb->m_segmentSize,
                                   (uint32_t)tcb->m_ssThresh);
        }
        else if (phase == HYSTART_CSS)
        {
            uint32_t limited = std::min(segmentsAcked, effectiveL);
            m_cssCwndAccum += limited * tcb->m_segmentSize;
            uint32_t increment = m_cssCwndAccum / cssGrowthDiv;
            if (increment > 0)
            {
                tcb->m_cWnd += increment;
                m_cssCwndAccum -= increment * cssGrowthDiv;
            }
        }
        else
        {
            TcpLinuxReno::IncreaseWindow(tcb, segmentsAcked);
        }
    }
    else
    {
        if (phase != HYSTART_CA)
        {
            phase = HYSTART_CA;
        }
        TcpLinuxReno::IncreaseWindow(tcb, segmentsAcked);
    }
}

uint32_t
TcpHyStartPlusPlus::GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
    if (phase == HYSTART_SS || phase == HYSTART_CSS)
    {
        uint32_t ssThresh = std::max(tcb->m_cWnd.Get(), 2 * tcb->m_segmentSize);
        phase = HYSTART_CA;
        m_isInitialSlowStart = false;
        return ssThresh;
    }
    return TcpLinuxReno::GetSsThresh(tcb, bytesInFlight);
}

void
TcpHyStartPlusPlus::CongestionStateSet(Ptr<TcpSocketState> tcb,
                                       const TcpSocketState::TcpCongState_t newState)
{
    if (newState == TcpSocketState::CA_LOSS)
    {
        phase = HYSTART_CA;
        m_isInitialSlowStart = false;
    }
}

Ptr<TcpCongestionOps>
TcpHyStartPlusPlus::Fork()
{
    return CopyObject<TcpHyStartPlusPlus>(this);
}

struct SimResult
{
    uint32_t nodeCount;
    uint32_t flowCount;
    uint32_t pps;
    double coverageMult;
    double throughputMbps;
    double avgDelayMs;
    double pdrPct;
    double dropRatioPct;
    double energyConsumedJ;
};

/**
 *
 * @param nodeCount        Total nodes (sources + sinks share the pool)
 * @param flowCount        Number of UDP flows
 * @param pps              Packets per second per flow
 * @param pktSize          Packet payload in bytes (fixed 1024)
 * @param txRangeMeters    WiFi Tx range (used to set RxGain / distance)
 * @param coverageMult     Coverage side = coverageMult × txRangeMeters
 * @param simTime          Simulation time in seconds
 */
SimResult
RunSimulation(uint32_t nodeCount,
              uint32_t flowCount,
              uint32_t pps,
              uint32_t pktSize,
              double txRangeMeters,
              double coverageMult,
              double simTime)
{
    SimResult res{nodeCount, flowCount, pps, coverageMult, 0, 0, 0, 0, 0};

    Config::SetDefault("ns3::TcpL4Protocol::SocketType", StringValue("ns3::TcpHyStartPlusPlus"));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));

    NodeContainer nodes;
    nodes.Create(nodeCount);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::RangePropagationLossModel",
                               "MaxRange",
                               DoubleValue(txRangeMeters));

    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("ErpOfdmRate24Mbps"),
                                 "ControlMode",
                                 StringValue("ErpOfdmRate6Mbps"));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    double coverageSide = coverageMult * txRangeMeters;
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    uint32_t cols = std::max(1u, (uint32_t)std::ceil(std::sqrt((double)nodeCount)));
    uint32_t rows = (uint32_t)std::ceil((double)nodeCount / cols);
    double stepX = (cols > 1) ? coverageSide / (cols - 1) : 0.0;
    double stepY = (rows > 1) ? coverageSide / (rows - 1) : 0.0;

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nodeCount; i++)
    {
        double x = (cols > 1) ? (i % cols) * stepX : coverageSide / 2.0;
        double y = (rows > 1) ? (i / cols) * stepY : coverageSide / 2.0;
        posAlloc->Add(Vector(x, y, 0.0));
    }
    mobility.SetPositionAllocator(posAlloc);
    mobility.Install(nodes);

    OlsrHelper olsr;
    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr);
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.0.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    BasicEnergySourceHelper energySrcHelper;
    energySrcHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100.0));
    energy::EnergySourceContainer energySources = energySrcHelper.Install(nodes);

    WifiRadioEnergyModelHelper radioEnergy;
    radioEnergy.Install(devices, energySources);

    // ---- Applications (UDP on-off) ----
    // Keep actual flow count ≤ nodeCount/2
    uint32_t actualFlows = std::min(flowCount, nodeCount / 2);
    uint32_t halfOffset = std::max(1u, nodeCount / 2);
    double appStart = 5.0;
    double appStop = simTime - 1.0;

    for (uint32_t i = 0; i < actualFlows; i++)
    {
        uint32_t src = i % nodeCount;
        uint32_t dst = (src + halfOffset) % nodeCount;
        if (dst == src)
        {
            dst = (dst + 1) % nodeCount;
        }

        uint16_t port = 50000 + i;

        // Sink
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(dst));
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(simTime));

        // Source — OnOff at fixed rate
        double dataRate = pps * pktSize * 8.0; // bps
        std::ostringstream rateStr;
        rateStr << (uint64_t)dataRate << "bps";

        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(ifaces.GetAddress(dst), port));
        onoff.SetConstantRate(DataRate(rateStr.str()), pktSize);
        ApplicationContainer srcApp = onoff.Install(nodes.Get(src));
        srcApp.Start(Seconds(appStart));
        srcApp.Stop(Seconds(appStop));
    }

    // ---- Flow monitor ----
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    auto& flowStats = monitor->GetFlowStats();

    uint64_t totalTx = 0, totalRx = 0, totalLost = 0, totalRxBytes = 0;
    double totalDelay = 0.0;

    for (auto& kv : flowStats)
    {
        totalTx += kv.second.txPackets;
        totalRx += kv.second.rxPackets;
        totalLost += kv.second.lostPackets;
        totalRxBytes += kv.second.rxBytes;
        totalDelay += kv.second.delaySum.GetMilliSeconds();
    }

    double duration = appStop - appStart;
    res.throughputMbps = (duration > 0) ? (totalRxBytes * 8.0 / duration / 1e6) : 0.0;
    res.avgDelayMs = (totalRx > 0) ? (totalDelay / totalRx) : 0.0;
    res.pdrPct = (totalTx > 0) ? (100.0 * totalRx / totalTx) : 0.0;
    res.dropRatioPct = (totalTx > 0) ? (100.0 * totalLost / totalTx) : 0.0;

    // Energy consumed = initial - remaining, summed over all nodes
    double energyConsumed = 0.0;
    for (uint32_t i = 0; i < energySources.GetN(); i++)
    {
        auto es = DynamicCast<energy::BasicEnergySource>(energySources.Get(i));
        if (es)
        {
            energyConsumed += 100.0 - es->GetRemainingEnergy();
        }    }
    res.energyConsumedJ = energyConsumed;

    Simulator::Destroy();
    return res;
}

// ============================================================================
//  Main — parameter sweep driver
// ============================================================================
int
main(int argc, char* argv[])
{
    double simTime = 40.0;
    double txRange = 50.0; // meters — WiFi Tx range
    uint32_t pktSize = 1024;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time per run (s)", simTime);
    cmd.AddValue("txRange", "WiFi Tx range in meters", txRange);
    cmd.Parse(argc, argv);

    std::vector<uint32_t> nodeCounts = {20, 40, 60, 80, 100};
    std::vector<uint32_t> flowCounts = {10, 20, 30, 40, 50};
    std::vector<uint32_t> ppsValues = {100, 200, 300, 400, 500};
    std::vector<double> covMults = {1.0, 2.0, 3.0, 4.0, 5.0};
    const uint32_t DEF_NODES = 20;
    const uint32_t DEF_FLOWS = 10;
    const uint32_t DEF_PPS = 100;
    const double DEF_COV = 1.0;

    auto openCsv = [](const std::string& name) {
        std::ofstream f(name);
        f << "varied_value,throughput_mbps,avg_delay_ms,pdr_pct,"
             "drop_ratio_pct,energy_J\n";
        return f;
    };

    std::cout << "===== HyStart++ WiFi 802.11 Static Sweep =====" << std::endl;
    std::cout << "TxRange=" << txRange << "m  simTime=" << simTime << "s  pktSize=" << pktSize
              << "B" << std::endl;

    {
        std::ofstream csv = openCsv("results_vary_nodes.csv");
        std::cout << "\n[1/4] Varying node count (flows=" << DEF_FLOWS << " pps=" << DEF_PPS
                  << " cov=" << DEF_COV << "x)" << std::endl;
        for (uint32_t n : nodeCounts)
        {
            std::cout << "  nodes=" << n << " ... " << std::flush;
            SimResult r = RunSimulation(n, DEF_FLOWS, DEF_PPS, pktSize, txRange, DEF_COV, simTime);
            csv << n << "," << r.throughputMbps << "," << r.avgDelayMs << "," << r.pdrPct << ","
                << r.dropRatioPct << "," << r.energyConsumedJ << "\n";
            std::cout << "throughput=" << r.throughputMbps << "Mbps  PDR=" << r.pdrPct
                      << "%  energy=" << r.energyConsumedJ << "J" << std::endl;
        }
    }

    // ---- 2. Vary flow count ----
    {
        std::ofstream csv = openCsv("results_vary_flows.csv");
        std::cout << "\n[2/4] Varying flow count (nodes=" << DEF_NODES << " pps=" << DEF_PPS
                  << " cov=" << DEF_COV << "x)" << std::endl;
        for (uint32_t f : flowCounts)
        {
            std::cout << "  flows=" << f << " ... " << std::flush;
            SimResult r = RunSimulation(DEF_NODES, f, DEF_PPS, pktSize, txRange, DEF_COV, simTime);
            csv << f << "," << r.throughputMbps << "," << r.avgDelayMs << "," << r.pdrPct << ","
                << r.dropRatioPct << "," << r.energyConsumedJ << "\n";
            std::cout << "throughput=" << r.throughputMbps << "Mbps  PDR=" << r.pdrPct
                      << "%  energy=" << r.energyConsumedJ << "J" << std::endl;
        }
    }

    // ---- 3. Vary packets per second ----
    {
        std::ofstream csv = openCsv("results_vary_pps.csv");
        std::cout << "\n[3/4] Varying pps (nodes=" << DEF_NODES << " flows=" << DEF_FLOWS
                  << " cov=" << DEF_COV << "x)" << std::endl;
        for (uint32_t p : ppsValues)
        {
            std::cout << "  pps=" << p << " ... " << std::flush;
            SimResult r =
                RunSimulation(DEF_NODES, DEF_FLOWS, p, pktSize, txRange, DEF_COV, simTime);
            csv << p << "," << r.throughputMbps << "," << r.avgDelayMs << "," << r.pdrPct << ","
                << r.dropRatioPct << "," << r.energyConsumedJ << "\n";
            std::cout << "throughput=" << r.throughputMbps << "Mbps  PDR=" << r.pdrPct
                      << "%  energy=" << r.energyConsumedJ << "J" << std::endl;
        }
    }

    // ---- 4. Vary coverage area ----
    {
        std::ofstream csv = openCsv("results_vary_coverage.csv");
        std::cout << "\n[4/4] Varying coverage (nodes=" << DEF_NODES << " flows=" << DEF_FLOWS
                  << " pps=" << DEF_PPS << ")" << std::endl;
        for (double c : covMults)
        {
            std::cout << "  coverage=" << c << "x (" << c * txRange << "m side) ... " << std::flush;
            SimResult r =
                RunSimulation(DEF_NODES, DEF_FLOWS, DEF_PPS, pktSize, txRange, c, simTime);
            csv << c << "," << r.throughputMbps << "," << r.avgDelayMs << "," << r.pdrPct << ","
                << r.dropRatioPct << "," << r.energyConsumedJ << "\n";
            std::cout << "throughput=" << r.throughputMbps << "Mbps  PDR=" << r.pdrPct
                      << "%  energy=" << r.energyConsumedJ << "J" << std::endl;
        }
    }

    std::cout << "\n===== All sweeps complete =====" << std::endl;
    std::cout << "Output files:" << std::endl;
    std::cout << "  results_vary_nodes.csv" << std::endl;
    std::cout << "  results_vary_flows.csv" << std::endl;
    std::cout << "  results_vary_pps.csv" << std::endl;
    std::cout << "  results_vary_coverage.csv" << std::endl;

    return 0;
}

/*
 * Run command:
 *   ./ns3 run "hystart_wifi_static"
 */
