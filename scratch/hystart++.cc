/*
 * Topology: Sender ---[10Mbps,5ms]--- Router ---[1Mbps,50ms]--- Receiver
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HyStartPlusPlusImpl");

static double g_bdpBytes = 0;

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
    ~TcpHyStartPlusPlus() override;

    std::string GetName() const override;
    void IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked) override;
    void PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt) override;
    uint32_t GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight) override;
    void CongestionStateSet(Ptr<TcpSocketState> tcb,
                            const TcpSocketState::TcpCongState_t newState) override;
    Ptr<TcpCongestionOps> Fork() override;

  private:
    Time minRttThresh;      ///< MIN_RTT_THRESH  = 4 ms
    Time maxRttThresh;      ///< MAX_RTT_THRESH  = 16 ms
    uint32_t minRttDivisor; ///< MIN_RTT_DIVISOR = 8
    uint32_t nRttSample;    ///< N_RTT_SAMPLE    = 8
    uint32_t cssGrowthDiv;  ///< CSS_GROWTH_DIVISOR = 4
    uint32_t cssMaxRounds;  ///< CSS_ROUNDS       = 5
    uint32_t m_L;

    HyStartPhase phase;
    bool roundStarted;
    SequenceNumber32 windowEnd;

    Time lastRoundMinRtt;
    Time currentRoundMinRtt;
    uint32_t rttSampleCount;
    std::vector<Time> lastRtt;
    Time m_cssBaselineMinRtt;
    uint32_t cssRoundCount;
    uint32_t m_cssCwndAccum;
    uint32_t windowTracker;
    bool m_isInitialSlowStart;

    uint32_t ssExitCount;
    uint32_t cssSpuriousCount;
    uint32_t ssExitCwnd;
    Time m_ssExitTime;

    void BeginNewRttRound(Ptr<TcpSocketState> tcb);

    /** RttThresh = max(MIN_RTT_THRESH, min(lastRoundMinRTT / MIN_RTT_DIVISOR, MAX_RTT_THRESH)) */
    Time ComputeRttThresh() const;

    /** Human-readable name for a phase */
    static const char* PhaseName(HyStartPhase p);
};

NS_OBJECT_ENSURE_REGISTERED(TcpHyStartPlusPlus);

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
      windowTracker(3),
      windowEnd(SequenceNumber32(0)),
      lastRoundMinRtt(Time::Max()),
      currentRoundMinRtt(Time::Max()),
      rttSampleCount(0),
      m_cssBaselineMinRtt(Time::Max()),
      cssRoundCount(0),
      m_cssCwndAccum(0),
      m_isInitialSlowStart(true),
      ssExitCount(0),
      cssSpuriousCount(0),
      ssExitCwnd(0),
      m_ssExitTime(Seconds(0)),
      lastRtt()
{
    NS_LOG_FUNCTION(this);
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
      m_cssBaselineMinRtt(sock.m_cssBaselineMinRtt),
      cssRoundCount(sock.cssRoundCount),
      m_cssCwndAccum(sock.m_cssCwndAccum),
      m_isInitialSlowStart(sock.m_isInitialSlowStart),
      ssExitCount(sock.ssExitCount),
      cssSpuriousCount(sock.cssSpuriousCount),
      ssExitCwnd(sock.ssExitCwnd),
      m_ssExitTime(sock.m_ssExitTime),
      lastRtt(sock.lastRtt)
{
    NS_LOG_FUNCTION(this);
}

TcpHyStartPlusPlus::~TcpHyStartPlusPlus()
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
    Time fraction = Time::FromDouble(lastRoundMinRtt.GetDouble() / (double)minRttDivisor, Time::NS);
    return std::max(minRttThresh, std::min(fraction, maxRttThresh));
}

void
TcpHyStartPlusPlus::PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt)
{
    NS_LOG_FUNCTION(this << tcb << segmentsAcked << rtt);

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
        std::cout << "HYSTART_INIT " << Simulator::Now().GetSeconds() << " windowEnd=" << windowEnd
                  << " cwnd=" << tcb->m_cWnd << std::endl;
    }
    lastRtt.push_back(rtt);
    if (lastRtt.size() > 3)
    {
        lastRtt.erase(lastRtt.begin());
    }
    Time weightedAvgRtt = Time::Max();
    if (lastRtt.size() == 3)
    {
        weightedAvgRtt =
            Time::FromDouble((lastRtt[0].GetDouble() * 0.2 + lastRtt[1].GetDouble() * 0.3 +
                              lastRtt[2].GetDouble() * 0.5),
                             Time::NS);
        NS_LOG_INFO("[ACKed] RTT=" << rtt.GetMilliSeconds()
                                   << "ms  weightedAvgRTT=" << weightedAvgRtt.GetMilliSeconds()
                                   << "ms  samples=" << lastRtt.size());
    }
    else
    {
        currentRoundMinRtt = std::min(currentRoundMinRtt, rtt);
        weightedAvgRtt = currentRoundMinRtt;
    }
    currentRoundMinRtt = weightedAvgRtt;
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

                std::cout << "HYSTART_PHASE " << Simulator::Now().GetSeconds()
                          << " CSS->CA reason=css_rounds_complete"
                          << " ssthresh=" << tcb->m_ssThresh << " cwnd=" << tcb->m_cWnd << " ("
                          << tcb->m_cWnd / tcb->m_segmentSize << " segs)" << std::endl;
                return;
            }
        }

        BeginNewRttRound(tcb);
    }

    currentRoundMinRtt = std::min(currentRoundMinRtt, rtt);
    rttSampleCount++;

    if (phase == HYSTART_SS)
    {
        if (rttSampleCount >= nRttSample && currentRoundMinRtt != Time::Max() &&
            lastRoundMinRtt != Time::Max())
        {
            Time rttThresh = ComputeRttThresh();
            Time delayTarget = lastRoundMinRtt + rttThresh;

            NS_LOG_INFO("[SS delay check] curMinRTT="
                        << currentRoundMinRtt.GetMilliSeconds()
                        << "ms  target=" << delayTarget.GetMilliSeconds()
                        << "ms  (lastMinRTT=" << lastRoundMinRtt.GetMilliSeconds()
                        << "ms + thresh=" << rttThresh.GetMilliSeconds() << "ms)"
                        << "  samples=" << rttSampleCount);

            if (currentRoundMinRtt >= delayTarget)
            {
                m_cssBaselineMinRtt = currentRoundMinRtt;
                phase = HYSTART_CSS;
                cssRoundCount = 0;
                m_cssCwndAccum = 0;
                ssExitCount++;
                ssExitCwnd = tcb->m_cWnd;
                m_ssExitTime = Simulator::Now();

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
            cssSpuriousCount++;

            std::cout << "HYSTART_PHASE " << Simulator::Now().GetSeconds()
                      << " CSS->SS reason=spurious_exit"
                      << " curMinRTT=" << currentRoundMinRtt.GetMilliSeconds() << "ms < cssBaseline"
                      << " cwnd=" << tcb->m_cWnd << " (" << tcb->m_cWnd / tcb->m_segmentSize
                      << " segs)" << std::endl;
        }
    }
}
