
/*
 *
 * This script demonstrates the key PROBLEMS with standard TCP Reno slow start:
 *   1. BDP OVERSHOOT  - cwnd grows far beyond Bandwidth-Delay Product
 *   2. PACKET LOSS    - aggressive growth causes queue overflow
 *   3. OVERSHOOTING   - cwnd exceeds what the network can handle
 *   4. RTT INFLATION  - queue buildup increases RTT (bufferbloat)
 *   5. THROUGHPUT LOSS - recovery after loss wastes time
 *

 * Topology: Sender(10Mbps,5ms) -- Router(1Mbps,50ms) -- Receiver
 *   BDP = 1Mbps × 100ms(RTT) = 12,500 bytes ≈ 23 segments
 *   But Reno's cwnd grows to 100,000+ bytes before loss!
 *
 * Output: Structured log lines for easy parsing and graphing
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tcp-header.h"
#include "ns3/traffic-control-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TcpRenoCheck");

static double g_bdpBytes = 0;
static uint32_t g_totalDrops = 0;

static void
TraceCwnd(uint32_t oldVal, uint32_t newVal)
{
    std::cout << "CWND " << Simulator::Now().GetSeconds() << " " << newVal << std::endl;

    if (g_bdpBytes > 0 && newVal > g_bdpBytes)
    {
        double overshootPct = ((double)newVal / g_bdpBytes - 1.0) * 100.0;
        std::cout << "BDP_OVERSHOOT " << Simulator::Now().GetSeconds() << " " << newVal << " "
                  << (uint32_t)g_bdpBytes << " " << overshootPct << std::endl;
    }
}

static void
TraceSsthresh(uint32_t oldVal, uint32_t newVal)
{
    std::cout << "SSTHRESH " << Simulator::Now().GetSeconds() << " " << newVal << std::endl;
}

static void
TraceRtt(Time oldVal, Time newVal)
{
    std::cout << "RTT " << Simulator::Now().GetSeconds() << " " << newVal.GetMilliSeconds()
              << std::endl;
}

static void
TraceCongState(TcpSocketState::TcpCongState_t oldState, TcpSocketState::TcpCongState_t newState)
{
    std::string names[] = {"CA_OPEN", "CA_DISORDER", "CA_CWR", "CA_RECOVERY", "CA_LOSS"};
    std::cout << "CONGSTATE " << Simulator::Now().GetSeconds() << " " << names[oldState] << " "
              << names[newState] << std::endl;
}

static void
TraceBytesInFlight(uint32_t oldVal, uint32_t newVal)
{
    std::cout << "BYTES_IN_FLIGHT " << Simulator::Now().GetSeconds() << " " << newVal << std::endl;
}

static void
TraceQueueDrop(Ptr<const QueueDiscItem> item)
{
    g_totalDrops++;
    std::cout << "QUEUE_DROP " << Simulator::Now().GetSeconds() << " " << item->GetSize() << " "
              << g_totalDrops << std::endl;
}

static void
ConnectSocketTraces()
{
    Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow",
                                  MakeCallback(&TraceCwnd));
    Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/SlowStartThreshold",
                                  MakeCallback(&TraceSsthresh));
    Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/RTT",
                                  MakeCallback(&TraceRtt));
    Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/CongState",
                                  MakeCallback(&TraceCongState));
    Config::ConnectWithoutContext("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/BytesInFlight",
                                  MakeCallback(&TraceBytesInFlight));
}

int
main(int argc, char* argv[])
{
    LogComponentEnable("TcpLinuxReno", LOG_LEVEL_INFO);
    LogComponentEnable("TcpRenoCheck", LOG_LEVEL_INFO);

    std::string bandwidth = "1Mbps";
    std::string delay = "50ms";
    uint32_t dataSize = 10000000; // 10 MB
    double simTime = 30.0;
    uint32_t queueSize = 50;

    CommandLine cmd(__FILE__);
    cmd.AddValue("bandwidth", "Bottleneck bandwidth", bandwidth);
    cmd.AddValue("delay", "Bottleneck delay (one-way)", delay);
    cmd.AddValue("data", "Data size in bytes", dataSize);
    cmd.AddValue("time", "Simulation time", simTime);
    cmd.AddValue("queue", "Queue size in packets", queueSize);
    cmd.Parse(argc, argv);

    DataRate bwRate(bandwidth);
    Time delayTime(delay);
    Time accessDelay("5ms");
    Time rtt = 2 * (accessDelay + delayTime);
    g_bdpBytes = bwRate.GetBitRate() * rtt.GetSeconds() / 8.0;

    std::cout << "===== TCP RENO DRAWBACK ANALYSIS =====" << std::endl;
    std::cout << "CONFIG bottleneck=" << bandwidth << " delay=" << delay
              << " RTT=" << rtt.GetMilliSeconds() << "ms"
              << " BDP=" << (uint32_t)g_bdpBytes << "bytes"
              << " BDP_segments=" << (uint32_t)(g_bdpBytes / 536) << " queue=" << queueSize
              << "pkts"
              << " data=" << dataSize << "bytes" << std::endl;
    std::cout << "BDP_LINE " << g_bdpBytes << std::endl;

    Config::SetDefault("ns3::TcpL4Protocol::SocketType", StringValue("ns3::TcpLinuxReno"));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));
    Config::SetDefault("ns3::PfifoFastQueueDisc::MaxSize",
                       QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS, queueSize)));

    NodeContainer nodes;
    nodes.Create(3);

    PointToPointHelper p2pAccess;
    p2pAccess.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    p2pAccess.SetChannelAttribute("Delay", StringValue("5ms"));

    PointToPointHelper p2pBottleneck;
    p2pBottleneck.SetDeviceAttribute("DataRate", StringValue(bandwidth));
    p2pBottleneck.SetChannelAttribute("Delay", StringValue(delay));

    NetDeviceContainer devices1 = p2pAccess.Install(nodes.Get(0), nodes.Get(1));
    NetDeviceContainer devices2 = p2pBottleneck.Install(nodes.Get(1), nodes.Get(2));

    InternetStackHelper stack;
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces1 = address.Assign(devices1);
    address.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces2 = address.Assign(devices2);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    Ipv4Address destAddr = interfaces2.GetAddress(1);
    BulkSendHelper source("ns3::TcpSocketFactory", InetSocketAddress(destAddr, 9));
    source.SetAttribute("MaxBytes", UintegerValue(dataSize));
    ApplicationContainer sourceApps = source.Install(nodes.Get(0));
    sourceApps.Start(Seconds(1.0));
    sourceApps.Stop(Seconds(simTime));

    PacketSinkHelper sink("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 9));
    ApplicationContainer sinkApps = sink.Install(nodes.Get(2));
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(simTime));

    TrafficControlHelper tch;
    tch.SetRootQueueDisc("ns3::PfifoFastQueueDisc");
    tch.Uninstall(devices2.Get(0));
    QueueDiscContainer qd = tch.Install(devices2.Get(0));
    qd.Get(0)->TraceConnectWithoutContext("Drop", MakeCallback(&TraceQueueDrop));

    Simulator::Schedule(Seconds(1.001), &ConnectSocketTraces);

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    auto stats = monitor->GetFlowStats();

    for (auto& entry : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(entry.first);
        double duration = entry.second.timeLastRxPacket.GetSeconds() -
                          entry.second.timeFirstTxPacket.GetSeconds();
        double throughput = (duration > 0) ? (entry.second.rxBytes * 8.0 / duration / 1e6) : 0;
        double lossRate = (entry.second.txPackets > 0)
                              ? ((double)entry.second.lostPackets / entry.second.txPackets * 100.0)
                              : 0;
        double avgDelay = (entry.second.rxPackets > 0)
                              ? (entry.second.delaySum.GetMilliSeconds() / entry.second.rxPackets)
                              : 0;
        double avgJitter =
            (entry.second.rxPackets > 1)
                ? (entry.second.jitterSum.GetMilliSeconds() / (entry.second.rxPackets - 1))
                : 0;

        std::cout << "FLOW " << entry.first << " " << t.sourceAddress << " -> "
                  << t.destinationAddress << std::endl;
        std::cout << "FLOW_STATS tx=" << entry.second.txPackets << " rx=" << entry.second.rxPackets
                  << " lost=" << entry.second.lostPackets << " lossRate=" << lossRate << "%"
                  << " throughput=" << throughput << "Mbps"
                  << " avgDelay=" << avgDelay << "ms"
                  << " avgJitter=" << avgJitter << "ms" << std::endl;
    }

    Simulator::Destroy();
    return 0;
}