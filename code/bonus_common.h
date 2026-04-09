#pragma once
/*
 * bonus_common.h

  */

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/tcp-linux-reno.h"
#include "ns3/tcp-socket-state.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ns3
{


class TcpHyStartPlusAdaptive : public TcpLinuxReno
{
  public:
    enum HyStartPhase
    {
        HYSTART_SS,
        HYSTART_CSS,
        HYSTART_CA
    };

    static TypeId GetTypeId();
    TcpHyStartPlusAdaptive();
    TcpHyStartPlusAdaptive(const TcpHyStartPlusAdaptive& sock);
    ~TcpHyStartPlusAdaptive() override = default;

    std::string GetName() const override
    {
        return "TcpHyStartPlusAdaptive";
    }

    void IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked) override;
    void PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt) override;
    uint32_t GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight) override;
    void CongestionStateSet(Ptr<TcpSocketState> tcb,
                            const TcpSocketState::TcpCongState_t newState) override;
    Ptr<TcpCongestionOps> Fork() override;

  private:
    Time m_minRttThresh;
    Time m_maxRttThresh;
    uint32_t m_baseDivisor; 
    uint32_t m_nRttSample;
    uint32_t m_cssGrowthDiv;
    uint32_t m_cssMaxRounds;
    uint32_t m_L;

    double m_alpha;
    uint32_t m_minDivisor;
    double m_lastCV;
    uint32_t m_adaptDivisor; 

    double m_rttSumNs;     
    double m_rttSumSqNs;   
    uint32_t m_rttVarCount; 

    HyStartPhase m_phase;
    bool m_roundStarted;
    SequenceNumber32 m_windowEnd;

    Time m_lastRoundMinRtt;
    Time m_currentRoundMinRtt;
    uint32_t m_rttSampleCount;

    // 3-sample weighted-average smoother (retained from original)
    static constexpr uint32_t RTT_WIN = 3;
    std::vector<Time> m_rttWindow;

    // CSS state
    Time m_cssBaseline;
    uint32_t m_cssRoundCount;
    uint32_t m_cssCwndAccum;
    bool m_isInitialSS;

    void BeginNewRttRound(Ptr<TcpSocketState> tcb);
    Time ComputeRttThresh() const;
};

NS_OBJECT_ENSURE_REGISTERED(TcpHyStartPlusAdaptive);

TypeId
TcpHyStartPlusAdaptive::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TcpHyStartPlusAdaptive")
            .SetParent<TcpLinuxReno>()
            .SetGroupName("Internet")
            .AddConstructor<TcpHyStartPlusAdaptive>()
            .AddAttribute("MinRttThresh",
                          "MIN_RTT_THRESH",
                          TimeValue(MilliSeconds(4)),
                          MakeTimeAccessor(&TcpHyStartPlusAdaptive::m_minRttThresh),
                          MakeTimeChecker())
            .AddAttribute("MaxRttThresh",
                          "MAX_RTT_THRESH",
                          TimeValue(MilliSeconds(16)),
                          MakeTimeAccessor(&TcpHyStartPlusAdaptive::m_maxRttThresh),
                          MakeTimeChecker())
            .AddAttribute("BaseDivisor",
                          "BASE_DIV (RFC MIN_RTT_DIVISOR)",
                          UintegerValue(8),
                          MakeUintegerAccessor(&TcpHyStartPlusAdaptive::m_baseDivisor),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("NRttSample",
                          "N_RTT_SAMPLE",
                          UintegerValue(8),
                          MakeUintegerAccessor(&TcpHyStartPlusAdaptive::m_nRttSample),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("CssGrowthDivisor",
                          "CSS_GROWTH_DIVISOR",
                          UintegerValue(4),
                          MakeUintegerAccessor(&TcpHyStartPlusAdaptive::m_cssGrowthDiv),
                          MakeUintegerChecker<uint32_t>(2))
            .AddAttribute("CssMaxRounds",
                          "CSS_ROUNDS",
                          UintegerValue(5),
                          MakeUintegerAccessor(&TcpHyStartPlusAdaptive::m_cssMaxRounds),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("L",
                          "Aggressiveness limit (8 = non-paced)",
                          UintegerValue(8),
                          MakeUintegerAccessor(&TcpHyStartPlusAdaptive::m_L),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("VartAlpha",
                          "VART scaling factor for CV  (default 1.5)",
                          DoubleValue(1.5),
                          MakeDoubleAccessor(&TcpHyStartPlusAdaptive::m_alpha),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("VartMinDivisor",
                          "VART minimum allowed adaptive divisor (default 3)",
                          UintegerValue(3),
                          MakeUintegerAccessor(&TcpHyStartPlusAdaptive::m_minDivisor),
                          MakeUintegerChecker<uint32_t>(1));
    return tid;
}

TcpHyStartPlusAdaptive::TcpHyStartPlusAdaptive()
    : TcpLinuxReno(),
      m_minRttThresh(MilliSeconds(4)),
      m_maxRttThresh(MilliSeconds(16)),
      m_baseDivisor(8),
      m_nRttSample(8),
      m_cssGrowthDiv(4),
      m_cssMaxRounds(5),
      m_L(8),
      m_alpha(1.5),
      m_minDivisor(3),
      m_lastCV(0.0),
      m_adaptDivisor(8),
      m_rttSumNs(0.0),
      m_rttSumSqNs(0.0),
      m_rttVarCount(0),
      m_phase(HYSTART_SS),
      m_roundStarted(false),
      m_windowEnd(SequenceNumber32(0)),
      m_lastRoundMinRtt(Time::Max()),
      m_currentRoundMinRtt(Time::Max()),
      m_rttSampleCount(0),
      m_rttWindow(),
      m_cssBaseline(Time::Max()),
      m_cssRoundCount(0),
      m_cssCwndAccum(0),
      m_isInitialSS(true)
{
}

TcpHyStartPlusAdaptive::TcpHyStartPlusAdaptive(const TcpHyStartPlusAdaptive& o)
    : TcpLinuxReno(o),
      m_minRttThresh(o.m_minRttThresh),
      m_maxRttThresh(o.m_maxRttThresh),
      m_baseDivisor(o.m_baseDivisor),
      m_nRttSample(o.m_nRttSample),
      m_cssGrowthDiv(o.m_cssGrowthDiv),
      m_cssMaxRounds(o.m_cssMaxRounds),
      m_L(o.m_L),
      m_alpha(o.m_alpha),
      m_minDivisor(o.m_minDivisor),
      m_lastCV(o.m_lastCV),
      m_adaptDivisor(o.m_adaptDivisor),
      m_rttSumNs(o.m_rttSumNs),
      m_rttSumSqNs(o.m_rttSumSqNs),
      m_rttVarCount(o.m_rttVarCount),
      m_phase(o.m_phase),
      m_roundStarted(o.m_roundStarted),
      m_windowEnd(o.m_windowEnd),
      m_lastRoundMinRtt(o.m_lastRoundMinRtt),
      m_currentRoundMinRtt(o.m_currentRoundMinRtt),
      m_rttSampleCount(o.m_rttSampleCount),
      m_rttWindow(o.m_rttWindow),
      m_cssBaseline(o.m_cssBaseline),
      m_cssRoundCount(o.m_cssRoundCount),
      m_cssCwndAccum(o.m_cssCwndAccum),
      m_isInitialSS(o.m_isInitialSS)
{
}

Ptr<TcpCongestionOps>
TcpHyStartPlusAdaptive::Fork()
{
    return CopyObject<TcpHyStartPlusAdaptive>(this);
}

void
TcpHyStartPlusAdaptive::BeginNewRttRound(Ptr<TcpSocketState> tcb)
{
    if (m_rttVarCount >= 2)
    {
        double mean = m_rttSumNs / m_rttVarCount;
        double var = m_rttSumSqNs / m_rttVarCount - mean * mean;
        double sd = std::sqrt(std::max(0.0, var));
        m_lastCV = (mean > 0.0) ? sd / mean : 0.0;

        double floatDiv = (double)m_baseDivisor / (1.0 + m_alpha * m_lastCV);
        m_adaptDivisor = std::max(m_minDivisor, (uint32_t)std::round(floatDiv));
        m_adaptDivisor = std::min(m_adaptDivisor, m_baseDivisor);

        NS_LOG_UNCOND("[VART] CV=" << m_lastCV << "  adaptDiv=" << m_adaptDivisor
                                   << " (base=" << m_baseDivisor << ")");
    }

    m_lastRoundMinRtt = m_currentRoundMinRtt;
    m_currentRoundMinRtt = Time::Max();
    m_rttSampleCount = 0;
    m_rttWindow.clear();

    m_rttSumNs = 0.0;
    m_rttSumSqNs = 0.0;
    m_rttVarCount = 0;

    m_windowEnd = tcb->m_highTxMark;
}

Time
TcpHyStartPlusAdaptive::ComputeRttThresh() const
{
    Time frac = Time::FromDouble(m_lastRoundMinRtt.GetDouble() / (double)m_adaptDivisor, Time::NS);
    return std::max(m_minRttThresh, std::min(frac, m_maxRttThresh));
}

void
TcpHyStartPlusAdaptive::PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time& rtt)
{
    if (rtt.IsZero() || rtt.IsNegative())
    {
        return;
    }
    if (!m_isInitialSS)
    {
        return;
    }
    if (m_phase == HYSTART_CA)
    {
        return;
    }

    if (!m_roundStarted)
    {
        m_windowEnd = tcb->m_highTxMark;
        m_roundStarted = true;
    }

    double rttNs = rtt.GetDouble();
    m_rttSumNs += rttNs;
    m_rttSumSqNs += rttNs * rttNs;
    m_rttVarCount++;

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
    }

    m_currentRoundMinRtt = std::min(m_currentRoundMinRtt, smoothed);
    m_rttSampleCount++;

    if (tcb->m_lastAckedSeq >= m_windowEnd)
    {
        if (m_phase == HYSTART_CSS)
        {
            m_cssRoundCount++;
            if (m_cssRoundCount >= m_cssMaxRounds)
            {
                m_phase = HYSTART_CA;
                tcb->m_ssThresh = tcb->m_cWnd;
                return;
            }
        }
        BeginNewRttRound(tcb);
    }

    if (m_phase == HYSTART_SS)
    {
        if (m_rttSampleCount >= m_nRttSample && m_currentRoundMinRtt != Time::Max() &&
            m_lastRoundMinRtt != Time::Max())
        {
            Time thresh = ComputeRttThresh();
            if (m_currentRoundMinRtt >= m_lastRoundMinRtt + thresh)
            {
                m_cssBaseline = m_currentRoundMinRtt;
                m_phase = HYSTART_CSS;
                m_cssRoundCount = 0;
                m_cssCwndAccum = 0;
            }
        }
    }
    else if (m_phase == HYSTART_CSS)
    {
        if (m_rttSampleCount >= m_nRttSample && m_currentRoundMinRtt < m_cssBaseline)
        {
            m_cssBaseline = Time::Max();
            m_phase = HYSTART_SS;
        }
    }
}

void
TcpHyStartPlusAdaptive::IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
    uint32_t effL = tcb->m_pacing ? UINT32_MAX : m_L;

    if (tcb->m_cWnd < tcb->m_ssThresh)
    {
        if (m_phase == HYSTART_SS)
        {
            uint32_t lim = std::min(segmentsAcked, effL);
            tcb->m_cWnd =
                std::min(tcb->m_cWnd.Get() + lim * tcb->m_segmentSize, (uint32_t)tcb->m_ssThresh);
        }
        else if (m_phase == HYSTART_CSS)
        {
            uint32_t lim = std::min(segmentsAcked, effL);
            m_cssCwndAccum += lim * tcb->m_segmentSize;
            uint32_t inc = m_cssCwndAccum / m_cssGrowthDiv;
            if (inc > 0)
            {
                tcb->m_cWnd += inc;
                m_cssCwndAccum -= inc * m_cssGrowthDiv;
            }
        }
        else
        {
            TcpLinuxReno::IncreaseWindow(tcb, segmentsAcked);
        }
    }
    else
    {
        if (m_phase != HYSTART_CA)
        {
            m_phase = HYSTART_CA;
        }
        TcpLinuxReno::IncreaseWindow(tcb, segmentsAcked);
    }
}

uint32_t
TcpHyStartPlusAdaptive::GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
    if (m_phase == HYSTART_SS || m_phase == HYSTART_CSS)
    {
        uint32_t sst = std::max(tcb->m_cWnd.Get(), 2 * tcb->m_segmentSize);
        m_phase = HYSTART_CA;
        m_isInitialSS = false;
        return sst;
    }
    return TcpLinuxReno::GetSsThresh(tcb, bytesInFlight);
}

void
TcpHyStartPlusAdaptive::CongestionStateSet(Ptr<TcpSocketState> tcb,
                                           const TcpSocketState::TcpCongState_t newState)
{
    if (newState == TcpSocketState::CA_LOSS)
    {
        m_phase = HYSTART_CA;
        m_isInitialSS = false;
    }
}

} 
