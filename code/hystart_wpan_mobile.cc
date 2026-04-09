/*

  *   ./ns3 run "hystart_wpan_mobile"
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/ipv6-flow-classifier.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/propagation-module.h"
#include "ns3/ripng-helper.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HyStartPlusPlusWpanMobile");

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
    TcpHyStartPlusPlus(const TcpHyStartPlusPlus& s);
    ~TcpHyStartPlusPlus() override = default;

    std::string GetName() const override
    {
        return "TcpHyStartPlusPlus";
    }

    static const char* PhaseName(HyStartPhase p)
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

    void IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segAcked) override;
    void PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segAcked, const Time& rtt) override;
    uint32_t GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight) override;
    void CongestionStateSet(Ptr<TcpSocketState> tcb,
                            const TcpSocketState::TcpCongState_t s) override;
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

    static constexpr uint32_t RTT_WIN = 3;
    std::vector<Time> m_rttWindow;

    Time m_cssBaselineMinRtt;
    uint32_t cssRoundCount;
    uint32_t m_cssCwndAccum;
    bool m_isInitialSS;

    void BeginNewRttRound(Ptr<TcpSocketState> tcb);
    Time ComputeRttThresh() const;
    void LogPhaseTransition(double t,
                            const char* from,
                            const char* to,
                            const char* reason,
                            uint32_t cwnd,
                            uint32_t segSz);
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
      m_isInitialSS(true)
{
}

void
TcpHyStartPlusPlus::LogPhaseTransition(double t,
                                       const char* from,
                                       const char* to,
                                       const char* reason,
                                       uint32_t cwnd,
                                       uint32_t segSz)
{
    std::cout << "HYSTART_PHASE " << t << " " << from << "->" << to << " reason=" << reason
              << " cwnd=" << cwnd << " segs=" << cwnd / segSz << std::endl;
    // dedicated file: time  from  to  reason  cwnd_bytes
    // if (g_phaseFile.is_open())
    // {
    //     g_phaseFile << t << " " << from << " " << to << " " << reason << " " << cwnd << "\n";
    // }
}

TcpHyStartPlusPlus::TcpHyStartPlusPlus(const TcpHyStartPlusPlus& s)
    : TcpLinuxReno(s),
      minRttThresh(s.minRttThresh),
      maxRttThresh(s.maxRttThresh),
      minRttDivisor(s.minRttDivisor),
      nRttSample(s.nRttSample),
      cssGrowthDiv(s.cssGrowthDiv),
      cssMaxRounds(s.cssMaxRounds),
      m_L(s.m_L),
      phase(s.phase),
      roundStarted(s.roundStarted),
      windowEnd(s.windowEnd),
      lastRoundMinRtt(s.lastRoundMinRtt),
      currentRoundMinRtt(s.currentRoundMinRtt),
      rttSampleCount(s.rttSampleCount),
      m_rttWindow(s.m_rttWindow),
      m_cssBaselineMinRtt(s.m_cssBaselineMinRtt),
      cssRoundCount(s.cssRoundCount),
      m_cssCwndAccum(s.m_cssCwndAccum),
      m_isInitialSS(s.m_isInitialSS)
{
}

Ptr<TcpCongestionOps>
TcpHyStartPlusPlus::Fork()
{
    return CopyObject<TcpHyStartPlusPlus>(this);
}

void
TcpHyStartPlusPlus::BeginNewRttRound(Ptr<TcpSocketState> tcb)
{
    lastRoundMinRtt = currentRoundMinRtt;
    currentRoundMinRtt = Time::Max();
    rttSampleCount = 0;
    m_rttWindow.clear();
    windowEnd = tcb->m_highTxMark;
    std::cout << "HYSTART_ROUND " << Simulator::Now().GetSeconds() << " phase=" << PhaseName(phase)
              << " lastMinRTT="
              << (lastRoundMinRtt == Time::Max() ? -1.0 : lastRoundMinRtt.GetMilliSeconds())
              << "ms windowEnd=" << windowEnd << " cwnd=" << tcb->m_cWnd << std::endl;
}

Time
TcpHyStartPlusPlus::ComputeRttThresh() const
{
    /*
     *   RttThresh = max(MIN_RTT_THRESH,
     *                   min(lastRoundMinRTT / MIN_RTT_DIVISOR, MAX_RTT_THRESH)) */

    Time frac = Time::FromDouble(lastRoundMinRtt.GetDouble() / (double)minRttDivisor, Time::NS);
    return std::max(minRttThresh, std::min(frac, maxRttThresh));
}

void
TcpHyStartPlusPlus::PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segAcked, const Time& rtt)
{
    NS_LOG_FUNCTION(this << tcb << segAcked << rtt);

    if (phase == HYSTART_CA && !m_isInitialSS)
    {
        return;
    }

    if (rtt.IsZero() || rtt.IsNegative())
    {
        return;
    }

    if (!roundStarted)
    {
        windowEnd = tcb->m_highTxMark;
        roundStarted = true;
        std::cout << "HYSTART_INIT " << Simulator::Now().GetSeconds() << " windowEnd=" << windowEnd
                  << " cwnd=" << tcb->m_cWnd << std::endl;
    }
    if (phase == HYSTART_CA)
    {
        return;
    }

    m_rttWindow.push_back(rtt);
    if (m_rttWindow.size() > RTT_WIN)
    {
        m_rttWindow.erase(m_rttWindow.begin());
    }

    Time smoothed = rtt;
    if (m_rttWindow.size() == RTT_WIN)
    {
        smoothed =
            Time::FromDouble(m_rttWindow[0].GetDouble() * 0.2 + m_rttWindow[1].GetDouble() * 0.3 +
                                 m_rttWindow[2].GetDouble() * 0.5,
                             Time::NS);
        NS_LOG_INFO("[ACKed] RTT=" << rtt.GetMilliSeconds()
                                   << "ms  weightedAvgRTT=" << smoothed.GetMilliSeconds()
                                   << "ms  samples=" << m_rttWindow.size());
    }

    currentRoundMinRtt = std::min(currentRoundMinRtt, smoothed);
    rttSampleCount++;

    if (tcb->m_lastAckedSeq >= windowEnd)
    {
        if (phase == HYSTART_CSS)
        {
            cssRoundCount++;
            std::cout << "HYSTART_CSS_ROUND " << Simulator::Now().GetSeconds()
                      << " round=" << cssRoundCount << "/" << cssMaxRounds
                      << " cwnd=" << tcb->m_cWnd << std::endl;

            if (cssRoundCount >= cssMaxRounds)
            {
                phase = HYSTART_CA;
                tcb->m_ssThresh = tcb->m_cWnd;
                LogPhaseTransition(Simulator::Now().GetSeconds(),
                                   "CSS",
                                   "CA",
                                   "css_rounds_complete",
                                   tcb->m_cWnd,
                                   tcb->m_segmentSize);
                m_isInitialSS = false;
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

            if (currentRoundMinRtt >= lastRoundMinRtt + ComputeRttThresh())
            {
                m_cssBaselineMinRtt = currentRoundMinRtt;
                phase = HYSTART_CSS;
                cssRoundCount = 0;
                m_cssCwndAccum = 0;
                LogPhaseTransition(Simulator::Now().GetSeconds(),
                                   "SS",
                                   "CSS",
                                   "delay_threshold_exceeded",
                                   tcb->m_cWnd,
                                   tcb->m_segmentSize);
                std::cout << "HYSTART_PHASE " << Simulator::Now().GetSeconds()
                          << " SS->CSS reason=delay_increase"
                          << " curMinRTT=" << currentRoundMinRtt.GetMilliSeconds()
                          << "ms >= target=" << delayTarget.GetMilliSeconds()
                          << "ms (lastMinRTT=" << lastRoundMinRtt.GetMilliSeconds()
                          << "ms + rttThresh=" << rttThresh.GetMilliSeconds() << "ms)"
                          << " cwnd=" << tcb->m_cWnd << " (" << tcb->m_cWnd / tcb->m_segmentSize
                          << " segs)"
                          << " cssBaseline=" << m_cssBaselineMinRtt.GetMilliSeconds() << "ms"
                          << std::endl;
            }
        }
    }

    else if (phase == HYSTART_CSS)
    {
        if (rttSampleCount >= nRttSample && currentRoundMinRtt < m_cssBaselineMinRtt)
        {
            m_cssBaselineMinRtt = Time::Max();
            phase = HYSTART_SS;
            LogPhaseTransition(Simulator::Now().GetSeconds(),
                               "CSS",
                               "SS",
                               "rtt_recovery",
                               tcb->m_cWnd,
                               tcb->m_segmentSize);
        }
    }
}

void
TcpHyStartPlusPlus::IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segAcked)
{
    uint32_t effL = tcb->m_pacing ? UINT32_MAX : m_L;
    if (tcb->m_cWnd < tcb->m_ssThresh)
    {
        if (phase == HYSTART_SS)
        {
            uint32_t lim = std::min(segAcked, effL);
            tcb->m_cWnd =
                std::min(tcb->m_cWnd.Get() + lim * tcb->m_segmentSize, (uint32_t)tcb->m_ssThresh);
        }
        else if (phase == HYSTART_CSS)
        {
            m_cssCwndAccum += std::min(segAcked, effL) * tcb->m_segmentSize;
            uint32_t inc = m_cssCwndAccum / cssGrowthDiv;
            if (inc > 0)
            {
                tcb->m_cWnd += inc;
                m_cssCwndAccum -= inc * cssGrowthDiv;
            }
        }
        else
        {
            TcpLinuxReno::IncreaseWindow(tcb, segAcked);
        }
    }
    else
    {
        if (phase != HYSTART_CA)
        {
            phase = HYSTART_CA;
        }
        TcpLinuxReno::IncreaseWindow(tcb, segAcked);
    }
}

uint32_t
TcpHyStartPlusPlus::GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bif)
{
    if (phase == HYSTART_SS || phase == HYSTART_CSS)
    {
        uint32_t sst = std::max(tcb->m_cWnd.Get(), 2 * tcb->m_segmentSize);
        phase = HYSTART_CA;
        LogPhaseTransition(Simulator::Now().GetSeconds(),
                           (phase == HYSTART_SS ? "SS" : "CSS"),
                           "CA",
                           "loss_detected",
                           tcb->m_cWnd,
                           tcb->m_segmentSize);
        return sst;
    }
    return TcpLinuxReno::GetSsThresh(tcb, bif);
}

void
TcpHyStartPlusPlus::CongestionStateSet(Ptr<TcpSocketState> tcb,
                                       const TcpSocketState::TcpCongState_t s)
{
    if (s == TcpSocketState::CA_LOSS)
    {
        phase = HYSTART_CA;
        m_isInitialSS = false;
    }
}

struct SimResult
{
    double variedValue{0};
    double throughputMbps{0};
    double avgDelayMs{0};
    double pdrPct{0};
    double dropPct{0};
    double energyJ{0};
};

static void
WriteWifiSingleCsv(const std::string& name,
                   uint32_t nodeCount,
                   uint32_t flowCount,
                   uint32_t pps,
                   uint32_t pktSize,
                   double txRange,
                   double coverageMult,
                   double simTime,
                   const SimResult& r)
{
    bool hasContent = false;
    {
        std::ifstream in(name);
        hasContent = in.good() && (in.peek() != std::ifstream::traits_type::eof());
    }

    std::ofstream f(name, std::ios::app);
    if (!hasContent)
    {
        f << "nodes,flows,pps,pkt_size,tx_range,coverage_mult,sim_time,throughput_mbps,avg_delay_"
             "ms,"
             "pdr_pct,drop_pct,energy_J\n";
    }
    f << nodeCount << "," << flowCount << "," << pps << "," << pktSize << "," << txRange << ","
      << coverageMult << "," << simTime << "," << r.throughputMbps << "," << r.avgDelayMs << ","
      << r.pdrPct << "," << r.dropPct << "," << r.energyJ << "\n";
}

static void
WriteWpanSingleCsv(const std::string& name,
                   uint32_t nodeCount,
                   uint32_t flowCount,
                   uint32_t pps,
                   uint32_t pktSize,
                   double speedMs,
                   double wpanArea,
                   double simTime,
                   const SimResult& r)
{
    bool hasContent = false;
    {
        std::ifstream in(name);
        hasContent = in.good() && (in.peek() != std::ifstream::traits_type::eof());
    }

    std::ofstream f(name, std::ios::app);
    if (!hasContent)
    {
        f << "nodes,flows,pps,pkt_size,speed_ms,wpan_area,sim_time,throughput_mbps,avg_delay_ms,"
             "pdr_"
             "pct,drop_pct,energy_J\n";
    }
    f << nodeCount << "," << flowCount << "," << pps << "," << pktSize << "," << speedMs << ","
      << wpanArea << "," << simTime << "," << r.throughputMbps << "," << r.avgDelayMs << ","
      << r.pdrPct << "," << r.dropPct << "," << r.energyJ << "\n";
}

/*
 */
static void
PrintWifiTopologyDiagram(uint32_t n, double txRange, double coverageMult)
{
    double side = coverageMult * txRange;
    uint32_t cols = std::max(1u, (uint32_t)std::ceil(std::sqrt((double)n)));
    uint32_t rows = (uint32_t)std::ceil((double)n / cols);
    std::cout << "\n╔══════════════════════════════════════════════════════════════════╗\n"
              << "║  TOPOLOGY 1 — IEEE 802.11g (Static)                             ║\n"
              << "╠══════════════════════════════════════════════════════════════════╣\n"
              << "║                                                                  ║\n"
              << "║  Coverage area  : " << std::setw(6) << std::fixed << std::setprecision(0)
              << side << " m × " << side << " m"
              << "                              ║\n"
              << "║  Tx range       : " << std::setw(4) << txRange << " m"
              << "                                             ║\n"
              << "║  Grid           : " << cols << " col × " << rows << " row  (" << n << " nodes)"
              << "                         ║\n"
              << "║  MAC/PHY        : AdhocWifiMac / YansWifiPhy (802.11g 24 Mbps)  ║\n"
              << "║  Routing        : OLSR (IPv4)                                   ║\n"
              << "║  TCP CC         : TcpHyStartPlusPlus (RFC 9406 + WA smoother)   ║\n"
              << "║                                                                  ║\n"
              << "║  ASCII (3×3 example, actual grid scales with nodeCount):         ║\n"
              << "║                                                                  ║\n"
              << "║  (0,side)  [N6]~~~[N7]~~~[N8]                                   ║\n"
              << "║             |      |      |      ~~~ = ad-hoc WiFi link          ║\n"
              << "║            [N3]~~~[N4]~~~[N5]        (within txRange)            ║\n"
              << "║             |      |      |                                      ║\n"
              << "║  (0,0)     [N0]~~~[N1]~~~[N2]   (fixed positions)               ║\n"
              << "║                                                                  ║\n"
              << "║  Flow pairing: Ni → N(i + nodeCount/2) % nodeCount              ║\n"
              << "║  Energy model: WifiRadioEnergyModel (per node, 100 J initial)    ║\n"
              << "╚══════════════════════════════════════════════════════════════════╝\n\n";
}

static SimResult
RunWifiStatic(uint32_t nodeCount,
              uint32_t flowCount,
              uint32_t pps,
              uint32_t pktSize,
              double txRange,
              double coverageMult,
              double simTime)
{
    SimResult res;
    res.variedValue = (double)nodeCount;

    Config::SetDefault("ns3::TcpL4Protocol::SocketType", StringValue("ns3::TcpHyStartPlusPlus"));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(655360)); // 640 KB
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(655360));

    NodeContainer nodes;
    nodes.Create(nodeCount);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(txRange));

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
    NetDeviceContainer devs = wifi.Install(phy, mac, nodes);

    double coverageSide = coverageMult * txRange;
    uint32_t cols = std::max(1u, (uint32_t)std::ceil(std::sqrt((double)nodeCount)));
    uint32_t rows = (uint32_t)std::ceil((double)nodeCount / cols);
    double sx = (cols > 1) ? coverageSide / (cols - 1) : 0.0;
    double sy = (rows > 1) ? coverageSide / (rows - 1) : 0.0;

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nodeCount; i++)
    {
        posAlloc->Add(Vector((cols > 1) ? (i % cols) * sx : coverageSide / 2.0,
                             (rows > 1) ? (i / cols) * sy : coverageSide / 2.0,
                             0.0));
    }

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.SetPositionAllocator(posAlloc);
    mob.Install(nodes);

    OlsrHelper olsr;
    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr);
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.0.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devs);

    // --- Energy ---
    BasicEnergySourceHelper esHelper;
    esHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100.0));
    energy::EnergySourceContainer energySrcs = esHelper.Install(nodes);
    WifiRadioEnergyModelHelper radioEnergy;
    radioEnergy.Install(devs, energySrcs);

    // --- Applications: Controlled-rate TCP ---
    double appStart = 5.0;
    double appStop = simTime - 1.0;
    uint32_t actualFlows = std::min(flowCount, nodeCount / 2);
    uint32_t offset = std::max(1u, nodeCount / 2);

    double packetInterval = 1.0 / pps;
    double onTime = pktSize * 8.0 / 1e9;
    std::ostringstream onStr, offStr;
    onStr << "ns3::ConstantRandomVariable[Constant=" << (pktSize * 8.0 / 1e9) << "]";
    offStr << "ns3::ConstantRandomVariable[Constant=" << (packetInterval - (pktSize * 8.0 / 1e9))
           << "]";

    std::vector<Ptr<PacketSink>> sinks;

    for (uint32_t i = 0; i < actualFlows; i++)
    {
        uint32_t src = i % nodeCount;
        uint32_t dst = (src + offset + i / nodeCount) % nodeCount;
        if (dst == src)
        {
            dst = (dst + 1) % nodeCount;
        }

        uint16_t port = 50000 + i;

        PacketSinkHelper sinkH("ns3::TcpSocketFactory",
                               InetSocketAddress(Ipv4Address::GetAny(), port));
        auto sinkApp = sinkH.Install(nodes.Get(dst));
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(simTime));
        sinks.push_back(DynamicCast<PacketSink>(sinkApp.Get(0)));

        OnOffHelper onoff("ns3::TcpSocketFactory", InetSocketAddress(ifaces.GetAddress(dst), port));
        onoff.SetAttribute("PacketSize", UintegerValue(pktSize));
        onoff.SetAttribute("OnTime", StringValue(onStr.str()));
        onoff.SetAttribute("OffTime", StringValue(offStr.str()));
        onoff.SetAttribute("DataRate",
                           DataRateValue(DataRate("100Mbps"))); // high enough not to limit

        auto srcApp = onoff.Install(nodes.Get(src));
        srcApp.Start(Seconds(appStart));
        srcApp.Stop(Seconds(appStop));
    }

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
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
    res.dropPct = (totalTx > 0) ? (100.0 * totalLost / totalTx) : 0.0;

    double energy = 0.0;
    for (uint32_t i = 0; i < energySrcs.GetN(); i++)
    {
        auto es = DynamicCast<energy::BasicEnergySource>(energySrcs.Get(i));
        if (es)
        {
            energy += 100.0 - es->GetRemainingEnergy();
        }
    }
    res.energyJ = energy;

    Simulator::Destroy();
    return res;
}

/*
 * IEEE 802.15.4 energy constants (CC2420 datasheet, Vsupply = 3.3 V)
 *   TX  : 17.4 mA  → 57.4 mW
 *   RX  : 18.8 mA  → 62.0 mW
 *   Idle: ~1.0 mA  →  3.3 mW
 */
static constexpr double WPAN_P_TX_W = 0.0574;   ///< TX power (W) — CC2420
static constexpr double WPAN_P_RX_W = 0.0620;   ///< RX power (W)
static constexpr double WPAN_P_IDLE_W = 0.0033; ///< Idle power (W)
static constexpr double WPAN_RATE_BPS = 250e3;  ///< LR-WPAN PHY rate (250 kbps)

static void
PrintWpanTopologyDiagram(uint32_t n, double speedMs)
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════════╗\n"
              << "║  TOPOLOGY 2 — IEEE 802.15.4 (Mobile)                           ║\n"
              << "╠══════════════════════════════════════════════════════════════════╣\n"
              << "║                                                                  ║\n"
              << "║  Nodes          : " << std::setw(3) << n
              << "  (random-waypoint mobility)                    ║\n"
              << "║  Speed          : " << std::setw(4) << std::fixed << std::setprecision(0)
              << speedMs << " m/s"
              << "                                                ║\n"
              << "║  PHY rate       : 250 kbps  (2.4 GHz O-QPSK)                   ║\n"
              << "║  MAC            : LR-WPAN CSMA/CA                               ║\n"
              << "║  Adaptation     : 6LoWPAN (SixLowPan)                           ║\n"
              << "║  Network layer  : IPv6                                           ║\n"
              << "║  Routing        : RIPng (IPv6 dynamic routing)                  ║\n"
              << "║  TCP CC         : TcpHyStartPlusPlus (RFC 9406 + WA smoother)   ║\n"
              << "║                                                                  ║\n"
              << "║  Mobility model: RandomWaypointMobilityModel                    ║\n"
              << "║   → nodes move at fixed speed, pause 2 s at each waypoint       ║\n"
              << "║   → deployment area: 200 m × 200 m                              ║\n"
              << "║                                                                  ║\n"
              << "║  ASCII snapshot (links change as nodes move):                    ║\n"
              << "║                                                                  ║\n"
              << "║  +---(200m)---+   ➜ = random-waypoint path                      ║\n"
              << "║  | N2  ➜  N5  |   ~ = active 802.15.4 link (CSMA/CA, ~30 m)    ║\n"
              << "║  |  ↗     ↘   |                                                 ║\n"
              << "║  | N0 ~~ N3   |   Flow pairing: Ni → N(i + N/2) % N            ║\n"
              << "║  |  ↘     ↗   |                                                 ║\n"
              << "║  | N1  ← N4   |   Energy: CC2420 power-state model              ║\n"
              << "║  +--------------+  TX=57.4mW  RX=62mW  Idle=3.3mW              ║\n"
              << "║                                                                  ║\n"
              << "╚══════════════════════════════════════════════════════════════════╝\n\n";
}

static SimResult
RunWpanMobile(uint32_t nodeCount,
              uint32_t flowCount,
              uint32_t pps,
              uint32_t pktSize,
              double speedMs,
              double areaSize,
              double simTime)
{
    SimResult res;

    Config::SetDefault("ns3::TcpL4Protocol::SocketType", StringValue("ns3::TcpHyStartPlusPlus"));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));

    NodeContainer nodes;
    nodes.Create(nodeCount);

    LrWpanHelper lrWpanHelper;

    NetDeviceContainer lrDevs = lrWpanHelper.Install(nodes);

    lrWpanHelper.CreateAssociatedPan(lrDevs, 0);

    SixLowPanHelper sixlowpan;
    NetDeviceContainer sixDevs = sixlowpan.Install(lrDevs);

    ObjectFactory speedVar;
    speedVar.SetTypeId("ns3::ConstantRandomVariable");
    speedVar.Set("Constant", DoubleValue(speedMs));

    ObjectFactory pauseVar;
    pauseVar.SetTypeId("ns3::ConstantRandomVariable");
    pauseVar.Set("Constant", DoubleValue(2.0)); // 2 s pause at each waypoint

    Ptr<ListPositionAllocator> initPos = CreateObject<ListPositionAllocator>();
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
    rng->SetAttribute("Min", DoubleValue(0.0));
    rng->SetAttribute("Max", DoubleValue(areaSize));
    for (uint32_t i = 0; i < nodeCount; i++)
    {
        initPos->Add(Vector(rng->GetValue(), rng->GetValue(), 0.0));
    }

    // Create waypoint allocator before using it to avoid dangling pointers
    Ptr<RandomRectanglePositionAllocator> waypointAlloc =
        CreateObject<RandomRectanglePositionAllocator>();
    waypointAlloc->SetAttribute("X",
                                StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" +
                                            std::to_string((int)areaSize) + "]"));
    waypointAlloc->SetAttribute("Y",
                                StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" +
                                            std::to_string((int)areaSize) + "]"));

    MobilityHelper mob;
    mob.SetPositionAllocator(initPos);
    mob.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                         "Speed",
                         PointerValue(speedVar.Create()->GetObject<RandomVariableStream>()),
                         "Pause",
                         PointerValue(pauseVar.Create()->GetObject<RandomVariableStream>()),
                         "PositionAllocator",
                         PointerValue(waypointAlloc));
    mob.Install(nodes);

    RipNgHelper ripNgRouting;
    InternetStackHelper internet;
    internet.SetRoutingHelper(ripNgRouting);
    internet.Install(nodes);

    Ipv6AddressHelper ipv6;
    ipv6.SetBase(Ipv6Address("2001:db8::"), Ipv6Prefix(64));
    Ipv6InterfaceContainer ifaces = ipv6.Assign(sixDevs);

    double appStart = 5.0;
    double appStop = simTime - 1.0;
    uint32_t actual = std::min(flowCount, nodeCount / 2);
    uint32_t offset = std::max(1u, nodeCount / 2);

    double reqRate = (double)pps * pktSize * 8.0;
    double dataRate = reqRate;
    std::ostringstream rateStr;
    rateStr << (uint64_t)dataRate << "bps";

    std::vector<Ptr<PacketSink>> sinks;
    for (uint32_t i = 0; i < actual; i++)
    {
        uint32_t src = i % nodeCount;
        uint32_t dst = (src + offset + i / nodeCount) % nodeCount;
        if (dst == src)
        {
            dst = (dst + 1) % nodeCount;
        }
        uint16_t port = 50000 + i;

        Inet6SocketAddress dstSock(ifaces.GetAddress(dst, 1), port);

        PacketSinkHelper sinkH("ns3::TcpSocketFactory",
                               Inet6SocketAddress(Ipv6Address::GetAny(), port));
        auto sinkApp = sinkH.Install(nodes.Get(dst));
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(simTime));
        sinks.push_back(DynamicCast<PacketSink>(sinkApp.Get(0)));

        OnOffHelper onoff("ns3::TcpSocketFactory", dstSock);
        onoff.SetConstantRate(DataRate((uint64_t)dataRate), pktSize);
        auto srcApp = onoff.Install(nodes.Get(src));
        srcApp.Start(Seconds(appStart));
        srcApp.Stop(Seconds(appStop));
    }

    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    auto& flowStats = monitor->GetFlowStats();

    uint64_t totalTx = 0, totalRx = 0, totalLost = 0, totalRxBytes = 0, totalTxBytes = 0;
    double totalDelay = 0.0;
    for (auto& kv : flowStats)
    {
        totalTx += kv.second.txPackets;
        totalRx += kv.second.rxPackets;
        totalLost += kv.second.lostPackets;
        totalRxBytes += kv.second.rxBytes;
        totalTxBytes += kv.second.txBytes;
        totalDelay += kv.second.delaySum.GetMilliSeconds();
    }

    double duration = appStop - appStart;
    res.throughputMbps = (duration > 0) ? (totalRxBytes * 8.0 / duration / 1e6) : 0.0;
    res.avgDelayMs = (totalRx > 0) ? (totalDelay / totalRx) : 0.0;
    res.pdrPct = (totalTx > 0) ? (100.0 * totalRx / totalTx) : 0.0;
    res.dropPct = (totalTx > 0) ? (100.0 * totalLost / totalTx) : 0.0;

    double totalBytes = (double)(totalTxBytes + totalRxBytes);
    double tActivePerN = (nodeCount > 0) ? totalBytes / nodeCount / WPAN_RATE_BPS : 0.0;
    tActivePerN = std::min(tActivePerN, duration);
    double tIdlePerN = std::max(0.0, duration - tActivePerN);
    double energyPerN = (WPAN_P_TX_W + WPAN_P_RX_W) * tActivePerN + WPAN_P_IDLE_W * tIdlePerN;
    res.energyJ = energyPerN * nodeCount;

    Simulator::Destroy();
    return res;
}

static std::ofstream
OpenCsv(const std::string& name, const std::string& xLabel)
{
    bool hasContent = false;
    {
        std::ifstream in(name);
        hasContent = in.good() && (in.peek() != std::ifstream::traits_type::eof());
    }

    std::ofstream f(name, std::ios::app);
    if (!hasContent)
    {
        f << xLabel << ",throughput_mbps,avg_delay_ms,pdr_pct,drop_pct,energy_J\n";
    }
    return f;
}

static void
WriteRow(std::ofstream& f, const SimResult& r)
{
    f << r.variedValue << "," << r.throughputMbps << "," << r.avgDelayMs << "," << r.pdrPct << ","
      << r.dropPct << "," << r.energyJ << "\n";
}

static void
PrintRow(const std::string& label, double val, const SimResult& r)
{
    std::cout << "  " << label << "=" << std::setw(5) << val << "  tp=" << std::fixed
              << std::setprecision(4) << r.throughputMbps << " Mbps  PDR=" << std::setprecision(1)
              << r.pdrPct << "%  delay=" << std::setprecision(2) << r.avgDelayMs
              << " ms  E=" << std::setprecision(3) << r.energyJ << " J\n";
}

int
main(int argc, char* argv[])
{
    std::string mode = "wpan";
    std::string vary = "all";
    bool singleRun = false;
    uint32_t nodeCount = 20;
    uint32_t flowCount = 10;
    uint32_t pps = 100;
    double coverageMult = 1.0;
    double speedMs = 5.0;
    double simTime = 40.0;
    double txRange = 50.0;   // WiFi Tx range (m)
    double wpanArea = 200.0; // 802.15.4 deployment area side (m)
    uint32_t pktSize = 1024;
    std::string wifiCsv = "wifi_vary_nodes.csv";
    std::string wpanCsv = "wpan_vary_speed.csv";

    CommandLine cmd(__FILE__);
    cmd.AddValue("mode", "Simulation mode: wpan", mode);
    cmd.AddValue("vary", "Sweep groups: all or comma-list (nodes,flows,pps,coverage,speed)", vary);
    cmd.AddValue("singleRun", "Run one exact configuration instead of sweeps", singleRun);
    cmd.AddValue("nodes", "Node count for singleRun", nodeCount);
    cmd.AddValue("flows", "Flow count for singleRun", flowCount);
    cmd.AddValue("pps", "Packets per second for singleRun", pps);
    cmd.AddValue("coverage", "WiFi coverage multiplier for singleRun", coverageMult);
    cmd.AddValue("speed", "WPAN speed in m/s for singleRun", speedMs);
    cmd.AddValue("simTime", "Simulation duration (s)", simTime);
    cmd.AddValue("txRange", "WiFi Tx range (m)", txRange);
    cmd.AddValue("wpanArea", "802.15.4 area side (m)", wpanArea);
    cmd.AddValue("wifiCsv", "WiFi CSV output file", wifiCsv);
    cmd.AddValue("wpanCsv", "WPAN CSV output file", wpanCsv);
    cmd.Parse(argc, argv);

    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    std::transform(vary.begin(), vary.end(), vary.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    if (mode != "wpan")
    {
        std::cerr << "Invalid --mode. This source supports wpan only." << std::endl;
        return 1;
    }

    auto hasToken = [&](const std::string& token) {
        std::stringstream ss(vary);
        std::string item;
        while (std::getline(ss, item, ','))
        {
            item.erase(std::remove_if(item.begin(),
                                      item.end(),
                                      [](unsigned char c) { return std::isspace(c) != 0; }),
                       item.end());
            if (item == token)
            {
                return true;
            }
        }
        return false;
    };

    bool runNodes = (vary == "all") || hasToken("nodes");
    bool runFlows = (vary == "all") || hasToken("flows");
    bool runPps = (vary == "all") || hasToken("pps");
    bool runCoverage = (vary == "all") || hasToken("coverage"); // static only
    bool runSpeed = (vary == "all") || hasToken("speed");       // mobility only

    // Mode-specific sweeps: speed applies only to WPAN, coverage only to WiFi.
    if (mode == "wifi")
    {
        runSpeed = false;
    }
    else if (mode == "wpan")
    {
        runCoverage = false;
    }

    if (!(runNodes || runFlows || runPps || runCoverage || runSpeed))
    {
        if (mode == "wifi")
        {
            std::cerr << "Invalid --vary for WiFi mode. Use all or a comma-list like "
                         "nodes,flows,pps,coverage."
                      << std::endl;
        }
        else if (mode == "wpan")
        {
            std::cerr << "Invalid --vary for WPAN mode. Use all or a comma-list like "
                         "nodes,flows,pps,speed."
                      << std::endl;
        }
        else
        {
            std::cerr << "Invalid --vary. Use all or a comma-list like "
                         "nodes,flows,pps,coverage,speed."
                      << std::endl;
        }
        return 1;
    }

    const uint32_t DEF_NODES = 20;
    const uint32_t DEF_FLOWS = 10;
    const uint32_t DEF_PPS = 100;
    const double DEF_COV = 1.0;
    const double DEF_SPEED = 5.0;

    std::vector<uint32_t> nodeSweep = {20, 40, 60, 80, 100};
    std::vector<uint32_t> flowSweep = {10, 20, 30, 40, 50};
    std::vector<uint32_t> ppsSweep = {100, 200, 300, 400, 500};
    std::vector<double> covSweep = {1.0, 2.0, 3.0, 4.0, 5.0};       // WiFi only
    std::vector<double> speedSweep = {5.0, 10.0, 15.0, 20.0, 25.0}; // 802.15.4 only

    if (singleRun)
    {
        if (mode == "wifi" || mode == "both")
        {
            PrintWifiTopologyDiagram(nodeCount, txRange, coverageMult);
            std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                      << "  WiFi 802.11g STATIC — single run\n"
                      << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";

            SimResult r =
                RunWifiStatic(nodeCount, flowCount, pps, pktSize, txRange, coverageMult, simTime);
            WriteWifiSingleCsv(wifiCsv,
                               nodeCount,
                               flowCount,
                               pps,
                               pktSize,
                               txRange,
                               coverageMult,
                               simTime,
                               r);
            PrintRow("wifi", 1.0, r);
            std::cout << "    " << wifiCsv << " (appended)\n";
        }

        if (mode == "wpan" || mode == "both")
        {
            PrintWpanTopologyDiagram(nodeCount, speedMs);
            std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                      << "  IEEE 802.15.4 MOBILE — single run\n"
                      << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";

            SimResult r =
                RunWpanMobile(nodeCount, flowCount, pps, pktSize, speedMs, wpanArea, simTime);
            WriteWpanSingleCsv(wpanCsv,
                               nodeCount,
                               flowCount,
                               pps,
                               pktSize,
                               speedMs,
                               wpanArea,
                               simTime,
                               r);
            PrintRow("wpan", 1.0, r);
            std::cout << "    " << wpanCsv << " (appended)\n";
        }

        std::cout << "\n===== Single run complete =====\n";
        return 0;
    }

    if (mode == "wifi" || mode == "both")
    {
        PrintWifiTopologyDiagram(DEF_NODES, txRange, DEF_COV);

        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                  << "  WiFi 802.11g STATIC — parameter sweeps\n"
                  << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";

        // ── 1a. Vary node count ──────────────────────────────────────────
        if (runNodes)
        {
            auto csv = OpenCsv("wifi_vary_nodes.csv", "node_count");
            std::cout << "\n[WiFi] Varying node count"
                      << " (flows=" << DEF_FLOWS << " pps=" << DEF_PPS << " cov=" << DEF_COV
                      << "×)\n";
            for (uint32_t n : nodeSweep)
            {
                SimResult r =
                    RunWifiStatic(n, DEF_FLOWS, DEF_PPS, pktSize, txRange, DEF_COV, simTime);
                r.variedValue = n;
                WriteRow(csv, r);
                PrintRow("wifi", 1.0, r);
                PrintRow("nodes", n, r);
            }
        }

        // ── 1b. Vary flow count ──────────────────────────────────────────
        if (runFlows)
        {
            auto csv = OpenCsv("wifi_vary_flows.csv", "flow_count");
            std::cout << "\n[WiFi] Varying flow count"
                      << " (nodes=" << DEF_NODES << " pps=" << DEF_PPS << " cov=" << DEF_COV
                      << "×)\n";
            for (uint32_t f : flowSweep)
            {
                SimResult r =
                    RunWifiStatic(DEF_NODES, f, DEF_PPS, pktSize, txRange, DEF_COV, simTime);
                r.variedValue = f;
                WriteRow(csv, r);
                PrintRow("flows", f, r);
            }
        }

        // ── 1c. Vary pps ─────────────────────────────────────────────────
        if (runPps)
        {
            auto csv = OpenCsv("wifi_vary_pps.csv", "pps");
            std::cout << "\n[WiFi] Varying packets/s"
                      << " (nodes=" << DEF_NODES << " flows=" << DEF_FLOWS << " cov=" << DEF_COV
                      << "×)\n";
            for (uint32_t p : ppsSweep)
            {
                SimResult r =
                    RunWifiStatic(DEF_NODES, DEF_FLOWS, p, pktSize, txRange, DEF_COV, simTime);
                r.variedValue = p;
                WriteRow(csv, r);
                PrintRow("pps", p, r);
            }
        }

        // ── 1d. Vary coverage area ────────────────────────────────────────
        if (runCoverage)
        {
            auto csv = OpenCsv("wifi_vary_coverage.csv", "coverage_mult");
            std::cout << "\n[WiFi] Varying coverage area"
                      << " (nodes=" << DEF_NODES << " flows=" << DEF_FLOWS << " pps=" << DEF_PPS
                      << ")\n";
            for (double c : covSweep)
            {
                SimResult r =
                    RunWifiStatic(DEF_NODES, DEF_FLOWS, DEF_PPS, pktSize, txRange, c, simTime);
                r.variedValue = c;
                WriteRow(csv, r);
                PrintRow("cov×", c, r);
            }
        }

        std::cout << "\n  WiFi outputs:\n";
        if (runNodes)
        {
            std::cout << "    wifi_vary_nodes.csv\n";
        }
        if (runFlows)
        {
            std::cout << "    wifi_vary_flows.csv\n";
        }
        if (runPps)
        {
            std::cout << "    wifi_vary_pps.csv\n";
        }
        if (runCoverage)
        {
            std::cout << "    wifi_vary_coverage.csv\n";
        }
    }

    if (mode == "wpan" || mode == "both")
    {
        PrintWpanTopologyDiagram(DEF_NODES, DEF_SPEED);

        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                  << "  IEEE 802.15.4 MOBILE — parameter sweeps\n"
                  << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";

        // ── 2a. Vary node count ──────────────────────────────────────────
        if (runNodes)
        {
            auto csv = OpenCsv("wpan_vary_nodes.csv", "node_count");
            std::cout << "\n[WPAN] Varying node count"
                      << " (flows=" << DEF_FLOWS << " pps=" << DEF_PPS << " speed=" << DEF_SPEED
                      << "m/s)\n";
            for (uint32_t n : nodeSweep)
            {
                SimResult r =
                    RunWpanMobile(n, DEF_FLOWS, DEF_PPS, pktSize, DEF_SPEED, wpanArea, simTime);
                r.variedValue = n;
                WriteRow(csv, r);
                PrintRow("nodes", n, r);
            }
        }

        // ── 2b. Vary flow count ──────────────────────────────────────────
        if (runFlows)
        {
            auto csv = OpenCsv("wpan_vary_flows.csv", "flow_count");
            std::cout << "\n[WPAN] Varying flow count"
                      << " (nodes=" << DEF_NODES << " pps=" << DEF_PPS << " speed=" << DEF_SPEED
                      << "m/s)\n";
            for (uint32_t f : flowSweep)
            {
                SimResult r =
                    RunWpanMobile(DEF_NODES, f, DEF_PPS, pktSize, DEF_SPEED, wpanArea, simTime);
                r.variedValue = f;
                WriteRow(csv, r);
                PrintRow("flows", f, r);
            }
        }

        // ── 2c. Vary pps ─────────────────────────────────────────────────
        if (runPps)
        {
            auto csv = OpenCsv("wpan_vary_pps.csv", "pps");
            std::cout << "\n[WPAN] Varying packets/s"
                      << " (nodes=" << DEF_NODES << " flows=" << DEF_FLOWS << " speed=" << DEF_SPEED
                      << "m/s)\n";
            for (uint32_t p : ppsSweep)
            {
                SimResult r =
                    RunWpanMobile(DEF_NODES, DEF_FLOWS, p, pktSize, DEF_SPEED, wpanArea, simTime);
                r.variedValue = p;
                WriteRow(csv, r);
                PrintRow("pps", p, r);
            }
        }

        // ── 2d. Vary node speed  (mobility-specific parameter) ───────────
        if (runSpeed)
        {
            auto csv = OpenCsv("wpan_vary_speed.csv", "speed_ms");
            std::cout << "\n[WPAN] Varying node speed"
                      << " (nodes=" << DEF_NODES << " flows=" << DEF_FLOWS << " pps=" << DEF_PPS
                      << ")\n";
            for (double s : speedSweep)
            {
                SimResult r =
                    RunWpanMobile(DEF_NODES, DEF_FLOWS, DEF_PPS, pktSize, s, wpanArea, simTime);
                r.variedValue = s;
                WriteRow(csv, r);
                PrintRow("speed", s, r);
            }
        }

        std::cout << "\n  WPAN outputs:\n";
        if (runNodes)
        {
            std::cout << "    wpan_vary_nodes.csv\n";
        }
        if (runFlows)
        {
            std::cout << "    wpan_vary_flows.csv\n";
        }
        if (runPps)
        {
            std::cout << "    wpan_vary_pps.csv\n";
        }
        if (runSpeed)
        {
            std::cout << "    wpan_vary_speed.csv        <- mobility sweep\n";
        }
    }

    std::cout << "\n===== All simulations complete =====\n";
    return 0;
}

/*
 * Run command:
 *   ./ns3 run "hystart_wpan_mobile"
 */
