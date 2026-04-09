/*
 *
 *
 * ────────────────────────────────────────
 * Topology:
 *
 *   [SrcA(0)] ─P2P(10Mbps,5ms)─ [SrcB(1)] ─P2P(10Mbps,5ms)─ [Gateway(2)]
 *                                                                   │
 *                                              WiFi 802.11g ad-hoc (static grid)
 *                                                                   │
 *                                              [WiFiNode3] … [WiFiNode(N+2)]
 *
  *
 * ────────────────────────────────────────────────────────
  *
 *   ./ns3 run bonus_a_hybrid
 */

#include "bonus_common.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BonusAHybrid");

static double
ComputeJFI(const std::vector<double>& throughputs)
{
    if (throughputs.empty())
    {
        return 0.0;
    }
    double sumX = 0.0, sumX2 = 0.0;
    for (double x : throughputs)
    {
        sumX += x;
        sumX2 += x * x;
    }
    double n = (double)throughputs.size();
    return (sumX2 > 0.0) ? (sumX * sumX) / (n * sumX2) : 1.0;
}

static std::ofstream g_queueFile;

static void
SampleQueueSize(Ptr<QueueDisc> qd)
{
    if (g_queueFile.is_open())
    {
        g_queueFile << Simulator::Now().GetSeconds() << " " << qd->GetNPackets() << "\n";
    }
    Simulator::Schedule(MilliSeconds(200), &SampleQueueSize, qd);
}

struct SimResult
{
    uint32_t variedParam;
    double throughputMbps;
    double avgDelayMs;
    double pdrPct;
    double dropPct;
    double energyJ;
    double jfi;
};

static SimResult
RunHybridSim(uint32_t wirelessN,
             uint32_t flowCount,
             uint32_t pps,
             uint32_t pktSize,
             double txRange,
             double coverageMult,
             double simTime,
             bool traceQueue = false)
{
    SimResult res{};

    Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                       StringValue("ns3::TcpHyStartPlusAdaptive"));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));

    const uint32_t WIRED_SRCS = 2;
    const uint32_t GW_IDX = WIRED_SRCS;                
    const uint32_t TOTAL = WIRED_SRCS + 1 + wirelessN; // +1 for gateway

    NodeContainer allNodes;
    allNodes.Create(TOTAL);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("5ms"));

    NetDeviceContainer p2pDev01 = p2p.Install(allNodes.Get(0), allNodes.Get(1));
    NetDeviceContainer p2pDev12 = p2p.Install(allNodes.Get(1), allNodes.Get(GW_IDX));

    double coverageSide = coverageMult * txRange;

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

    NodeContainer wifiNodes;
    wifiNodes.Add(allNodes.Get(GW_IDX));
    for (uint32_t i = WIRED_SRCS + 1; i < TOTAL; i++)
    {
        wifiNodes.Add(allNodes.Get(i));
    }

    NetDeviceContainer wifiDevs = wifi.Install(phy, mac, wifiNodes);

        MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    Ptr<ListPositionAllocator> wiredPos = CreateObject<ListPositionAllocator>();
    wiredPos->Add(Vector(-100.0, 0.0, 0.0)); // SrcA
    wiredPos->Add(Vector(-50.0, 0.0, 0.0));  // SrcB
    wiredPos->Add(Vector(0.0, 0.0, 0.0));    // Gateway
    mob.SetPositionAllocator(wiredPos);
    mob.Install(allNodes.Get(0));
    mob.Install(allNodes.Get(1));
    mob.Install(allNodes.Get(GW_IDX));

    uint32_t wifiCount = wirelessN + 1;
    uint32_t cols = std::max(1u, (uint32_t)std::ceil(std::sqrt((double)wifiCount)));
    uint32_t rows = (uint32_t)std::ceil((double)wifiCount / cols);
    double sx = (cols > 1) ? coverageSide / (cols - 1) : 0.0;
    double sy = (rows > 1) ? coverageSide / (rows - 1) : 0.0;

    Ptr<ListPositionAllocator> wifiPos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < wifiCount; i++)
    {
        double x = (cols > 1) ? (i % cols) * sx : coverageSide / 2.0;
        double y = (rows > 1) ? (i / cols) * sy : coverageSide / 2.0;
        wifiPos->Add(Vector(x, y, 0.0));
    }
    mob.SetPositionAllocator(wifiPos);
    for (uint32_t i = 0; i < wifiNodes.GetN(); i++)
    {
        mob.Install(wifiNodes.Get(i));
    }

    OlsrHelper olsr;
    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr);
    internet.Install(allNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(p2pDev01);

    ipv4.SetBase("10.1.2.0", "255.255.255.0");
    ipv4.Assign(p2pDev12);

    ipv4.SetBase("10.2.0.0", "255.255.0.0");
    Ipv4InterfaceContainer wifiIfaces = ipv4.Assign(wifiDevs);
    // wifiIfaces[0] = gateway WiFi addr; wifiIfaces[1..] = wireless nodes

    TrafficControlHelper tch;
    Ptr<QueueDisc> gwQd = nullptr;
    Ptr<TrafficControlLayer> tc = allNodes.Get(GW_IDX)->GetObject<TrafficControlLayer>();
    if (tc)
    {
        gwQd = tc->GetRootQueueDiscOnDevice(wifiDevs.Get(0));
    }
    if (!gwQd)
    {
        tch.SetRootQueueDisc("ns3::FqCoDelQueueDisc");
        QueueDiscContainer installed = tch.Install(wifiDevs.Get(0)); // gateway WiFi dev
        gwQd = installed.Get(0);
    }

    if (traceQueue)
    {
        g_queueFile.open("queue_size_hybrid.dat", std::ios::trunc);
        g_queueFile << "# time(s)  queue_packets\n";
        Simulator::Schedule(MilliSeconds(200), &SampleQueueSize, gwQd);
    }

    BasicEnergySourceHelper esHelper;
    esHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100.0));
    energy::EnergySourceContainer energySrcs = esHelper.Install(wifiNodes);

    WifiRadioEnergyModelHelper radioEnergy;
    radioEnergy.Install(wifiDevs, energySrcs);

    // ── Applications ─────────────────────────────────────────────────────
    double appStart = 5.0;
    double appStop = simTime - 1.0;

    uint32_t actualFlows = std::min(flowCount, (uint32_t)(wifiNodes.GetN() - 1));

    std::vector<Ptr<PacketSink>> sinkPtrs;

    for (uint32_t i = 0; i < actualFlows; i++)
    {
        uint16_t port = 49000 + i;

        uint32_t dstWifiIdx = 1 + (i % wirelessN); // wifiNodes index
        Ptr<Node> dstNode = wifiNodes.Get(dstWifiIdx);
        Ipv4Address dstAddr = wifiIfaces.GetAddress(dstWifiIdx);

        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkApp = sinkHelper.Install(dstNode);
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(simTime));
        sinkPtrs.push_back(DynamicCast<PacketSink>(sinkApp.Get(0)));

             Ptr<Node> srcNode;
        if (i % 3 != 2)
        {
            srcNode = allNodes.Get(i % WIRED_SRCS); // SrcA or SrcB  (cross-type!)
        }
        else
        {
            uint32_t srcWifiIdx = 1 + ((i + wirelessN / 2) % wirelessN);
            if (srcWifiIdx == dstWifiIdx)
            {
                srcWifiIdx = 1 + (srcWifiIdx % wirelessN);
            }
            srcNode = wifiNodes.Get(srcWifiIdx);
        }

        double dataRate = (double)pps * pktSize * 8.0;
        std::ostringstream rateStr;
        rateStr << (uint64_t)dataRate << "bps";

        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
        onoff.SetConstantRate(DataRate(rateStr.str()), pktSize);
        ApplicationContainer srcApp = onoff.Install(srcNode);
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
    std::vector<double> flowThroughputs;

    double duration = appStop - appStart;
    for (auto& kv : flowStats)
    {
        totalTx += kv.second.txPackets;
        totalRx += kv.second.rxPackets;
        totalLost += kv.second.lostPackets;
        totalRxBytes += kv.second.rxBytes;
        totalDelay += kv.second.delaySum.GetMilliSeconds();

        double flowDur =
            kv.second.timeLastRxPacket.GetSeconds() - kv.second.timeFirstTxPacket.GetSeconds();
        if (flowDur > 0)
        {
            flowThroughputs.push_back(kv.second.rxBytes * 8.0 / flowDur / 1e6);
        }
    }

    res.throughputMbps = (duration > 0) ? (totalRxBytes * 8.0 / duration / 1e6) : 0.0;
    res.avgDelayMs = (totalRx > 0) ? (totalDelay / totalRx) : 0.0;
    res.pdrPct = (totalTx > 0) ? (100.0 * totalRx / totalTx) : 0.0;
    res.dropPct = (totalTx > 0) ? (100.0 * totalLost / totalTx) : 0.0;
    res.jfi = ComputeJFI(flowThroughputs);

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

    if (traceQueue) 
    {
        std::ofstream pnFile("per_node_throughput_hybrid.csv", std::ios::trunc);
        pnFile << "node_idx,rx_bytes,throughput_mbps\n";
        for (uint32_t i = 0; i < sinkPtrs.size(); i++)
        {
            if (sinkPtrs[i])
            {
                uint64_t rxB = sinkPtrs[i]->GetTotalRx();
                double tput = (duration > 0) ? rxB * 8.0 / duration / 1e6 : 0.0;
                pnFile << i << "," << rxB << "," << tput << "\n";
            }
        }
        if (g_queueFile.is_open())
        {
            g_queueFile.close();
        }
    }

    Simulator::Destroy();
    return res;
}



int
main(int argc, char* argv[])
{
    double simTime = 40.0;
    double txRange = 50.0;
    uint32_t pktSize = 1024;
    bool doFullSweep = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("txRange", "WiFi Tx range (m)", txRange);
    cmd.AddValue("fullSweep", "Run all 4 sweeps (1=yes)", doFullSweep);
    cmd.Parse(argc, argv);

    // std::cout << "===== Bonus A: Hybrid Wired + WiFi (HyStart++ VART) =====\n"
    //           << "Topology: [SrcA]--P2P--[SrcB]--P2P--[GW]~~~WiFi~~~[WirelessNodes]\n"
    //           << "Cross-type flows: wired SrcA/SrcB → wireless destinations\n\n";

    const uint32_t DEF_NODES = 20;
    const uint32_t DEF_FLOWS = 10;
    const uint32_t DEF_PPS = 100;
    const double DEF_COV = 1.0;

    std::vector<uint32_t> nodeSweep = {20, 40, 60, 80, 100};
    std::vector<uint32_t> flowSweep = {10, 20, 30, 40, 50};
    std::vector<uint32_t> ppsSweep = {100, 200, 300, 400, 500};
    std::vector<double> covSweep = {1.0, 2.0, 3.0, 4.0, 5.0};

    auto openCsv = [](const std::string& name) {
        std::ofstream f(name);
        f << "varied_value,throughput_mbps,avg_delay_ms,pdr_pct,"
             "drop_pct,energy_J,jains_fairness_index\n";
        return f;
    };

    auto printResult = [](const char* label, uint32_t val, const SimResult& r) {
        // std::cout << "  " << label << "=" << val << "  tp=" << std::fixed << std::setprecision(3)
        //           << r.throughputMbps << "Mbps  PDR=" << std::setprecision(1) << r.pdrPct
        //           << "%  JFI=" << std::setprecision(3) << r.jfi << "  E=" << std::setprecision(2)
        //           << r.energyJ << "J\n";
    };

    // ── 1. Vary wireless node count ──────────────────────────────────────
    {
        std::ofstream csv = openCsv("results_hybrid_vary_nodes.csv");
        for (uint32_t n : nodeSweep)
        {
            bool trace = (n == nodeSweep.front()); // capture queue only on first run
            SimResult r =
                RunHybridSim(n, DEF_FLOWS, DEF_PPS, pktSize, txRange, DEF_COV, simTime, trace);
            csv << n << "," << r.throughputMbps << "," << r.avgDelayMs << "," << r.pdrPct << ","
                << r.dropPct << "," << r.energyJ << "," << r.jfi << "\n";
            printResult("nodes", n, r);
        }
    }
    if (!doFullSweep)
    {
        // std::cout << "(--fullSweep=0: skipping remaining sweeps)\n";
        return 0;
    }

    // ── 2. Vary flow count ────────────────────────────────────────────────
    {
        std::ofstream csv = openCsv("results_hybrid_vary_flows.csv");
        // std::cout << "[2/4] Varying flow count\n";
        for (uint32_t f : flowSweep)
        {
            SimResult r =
                RunHybridSim(DEF_NODES, f, DEF_PPS, pktSize, txRange, DEF_COV, simTime, false);
            csv << f << "," << r.throughputMbps << "," << r.avgDelayMs << "," << r.pdrPct << ","
                << r.dropPct << "," << r.energyJ << "," << r.jfi << "\n";
            printResult("flows", f, r);
        }
    }

    // ── 3. Vary packets per second ────────────────────────────────────────
    {
        std::ofstream csv = openCsv("results_hybrid_vary_pps.csv");
        // std::cout << "[3/4] Varying pps\n";
        for (uint32_t p : ppsSweep)
        {
            SimResult r =
                RunHybridSim(DEF_NODES, DEF_FLOWS, p, pktSize, txRange, DEF_COV, simTime, false);
            csv << p << "," << r.throughputMbps << "," << r.avgDelayMs << "," << r.pdrPct << ","
                << r.dropPct << "," << r.energyJ << "," << r.jfi << "\n";
            printResult("pps", p, r);
        }
    }

    // ── 4. Vary coverage ─────────────────────────────────────────────────
    {
        std::ofstream csv = openCsv("results_hybrid_vary_coverage.csv");
        // std::cout << "[4/4] Varying coverage multiplier\n";
        for (double c : covSweep)
        {
            SimResult r =
                RunHybridSim(DEF_NODES, DEF_FLOWS, DEF_PPS, pktSize, txRange, c, simTime, false);
            uint32_t cInt = (uint32_t)(c * 10);
            csv << c << "," << r.throughputMbps << "," << r.avgDelayMs << "," << r.pdrPct << ","
                << r.dropPct << "," << r.energyJ << "," << r.jfi << "\n";
            printResult("cov×10", cInt, r);
        }
    }

    // std::cout << "\nOutputs:\n"
    //           << "  results_hybrid_vary_{nodes,flows,pps,coverage}.csv\n"
    //           << "  queue_size_hybrid.dat        (queue over time — Bonus C)\n"
    //           << "  per_node_throughput_hybrid.csv (per-node — Bonus C)\n";
    return 0;
}
