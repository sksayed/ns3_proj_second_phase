#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mesh-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/animation-interface.h"
#include "ns3/csma-module.h"
#include "ns3/buildings-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/config.h"
#include "ns3/wifi-net-device.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/node-list.h"
#include "ns3/mac48-address.h"
#include "ns3/simulator.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/wifi-mac-header.h"

// LTE/EPC overlay for hybrid UEs
#include "ns3/lte-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/hybrid-buildings-propagation-loss-model.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/ideal-beamforming-algorithm.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <deque>
#include <algorithm>
#include <cctype>
#include <map>
#include <limits>
#include <iostream>
#include <filesystem>

using namespace ns3;
namespace fs = std::filesystem;
                                                                    
NS_LOG_COMPONENT_DEFINE("MeshSimulationHybridTry2");

struct MeshAPDeviceConfig
{
    std::string name;                    // Device name
    std::string description;             // Use case description
    
    // Physical layer parameters
    double txPowerStart;                 // TX power start (dBm) - for mesh backhaul
    double txPowerEnd;                   // TX power end (dBm) - for mesh backhaul
    double hotspotTxPower;               // TX power for hotspot/AP mode (dBm) - separate radio
    double rxSensitivity;                // RX sensitivity (dBm)
    double rxGain;                       // RX antenna gain (dB)
    double txGain;                       // TX antenna gain (dB)
    
    // WiFi configuration
    WifiStandard wifiStandard;           // WiFi standard enum
    std::string dataMode;                // Data mode (e.g., "VhtMcs8")
    uint32_t numInterfaces;              // Number of mesh interfaces
    
    // Topology parameters (for 250m spacing in 3x3 grid)
    double meshRange;                    // Expected mesh range (m)
    
    // Constructor
    MeshAPDeviceConfig(std::string n, std::string desc, double txStart, double txEnd,
                      double hotspotTx, double rxSens, double rxG, double txG, 
                      WifiStandard standard, std::string mode, uint32_t interfaces, double range)
        : name(n), description(desc), txPowerStart(txStart), txPowerEnd(txEnd),
          hotspotTxPower(hotspotTx), rxSensitivity(rxSens), rxGain(rxG), txGain(txG), 
          wifiStandard(standard), dataMode(mode), numInterfaces(interfaces),
          meshRange(range) {}
};

/*
 * Factory function to get mesh device configuration by ID.
 * All profiles are configured for a 2.4GHz MESH interface (txPowerStart)
 * and a 5GHz HOTSPOT interface (hotspotTxPower).
 */
MeshAPDeviceConfig GetMeshDeviceConfig(uint32_t configId)
{
    switch(configId) {
        case 1:
            return MeshAPDeviceConfig(
                "TP-Link EAP225-Outdoor",
                "802.11n mesh @ 2.4GHz backhaul, 5GHz hotspot",
                23.0,                           // txPowerStart (dBm) - Verified 2.4GHz spec
                23.0,                           // txPowerEnd (dBm)
                22.0,                           // hotspotTxPower (dBm) - Verified 5GHz spec
                -90.0,                          // rxSensitivity (dBm) - Verified @ lowest rate
                5.0,                            // rxGain (dB) - External antenna
                5.0,                            // txGain (dB) - External antenna
                WIFI_STANDARD_80211n,           // wifiStandard (2.4 GHz, 802.11s compatible)
                "HtMcs7",                       // dataMode (802.11n)
                1,                              // numInterfaces
                300.0                           // meshRange (m)
            );
        
        case 2:
            return MeshAPDeviceConfig(
                "Netgear Orbi 960 (WiFi 6E)",
                "Premium indoor mesh (802.11ax @ 2.4GHz mesh, 12 antennas, WiFi 6E)",
                20.0,                           // txPowerStart (dBm) - FCC indoor limit
                20.0,                           // txPowerEnd (dBm)
                20.0,                           // hotspotTxPower (dBm) - 5GHz hotspot
                -92.0,                          // rxSensitivity (dBm) - Better due to more antennas
                3.0,                            // rxGain (dB) - 12 antennas = better gain
                3.0,                            // txGain (dB)
                WIFI_STANDARD_80211ax,          // wifiStandard (WiFi 6E)
                "HeMcs9",                       // dataMode (High MCS - premium device)
                1,                              // numInterfaces
                120.0                           // meshRange (m)
            );
        
        case 3:
            return MeshAPDeviceConfig(
                "ASUS ZenWiFi AX (XT8)",
                "Mid-range indoor mesh (802.11ax @ 2.4GHz mesh, 6 antennas, WiFi 6)",
                20.0,                           // txPowerStart (dBm) - FCC indoor limit
                20.0,                           // txPowerEnd (dBm)
                20.0,                           // hotspotTxPower (dBm) - 5GHz hotspot
                -88.0,                          // rxSensitivity (dBm) - Standard WiFi 6
                2.0,                            // rxGain (dB) - 6 antennas = standard gain
                2.0,                            // txGain (dB)
                WIFI_STANDARD_80211ax,          // wifiStandard (WiFi 6)
                "HeMcs7",                       // dataMode (Medium MCS - mid-range device)
                1,                              // numInterfaces
                100.0                           // meshRange (m)
            );
        
        case 0:
        default:
            // Default: Current configuration from wifi-test-2-adhoc-grid.cc
            // 802.11g, 2.4 GHz, medium power for testing
            return MeshAPDeviceConfig(
                "Default Test Configuration",
                "Current 802.11g mesh for testing (2.4 GHz)",
                20.0,                           // txPowerStart (dBm) - default mesh backhaul
                20.0,                           // txPowerEnd (dBm)
                20.0,                           // hotspotTxPower (dBm) - default hotspot (same as mesh)
                -96.0,                          // rxSensitivity (dBm) - 802.11g typical
                0.0,                            // rxGain (dB) - no antenna gain
                0.0,                            // txGain (dB)
                WIFI_STANDARD_80211g,           // WiFi standard - current default
                "ErpOfdmRate54Mbps",            // Data mode (802.11g)
                1,                              // numInterfaces - single band
                250.0                           // meshRange (m)
            );
    }
}

struct MeshNetworkConfig
{
    NodeContainer meshNodes;
    NetDeviceContainer meshDevices;
    Ipv4InterfaceContainer meshInterfaces;
    YansWifiPhyHelper wifiPhy;
};

struct InternetConfig
{
    NodeContainer internetNodes;
    NetDeviceContainer backhaulDevices;
    NetDeviceContainer backboneDevices;
    Ipv4InterfaceContainer backhaulInterfaces;
    Ipv4InterfaceContainer internetInterfaces;
};

struct HotspotConfig
{
    NodeContainer staNodes;
    NetDeviceContainer apDevices;
    NetDeviceContainer staDevices;
    Ipv4InterfaceContainer apInterfaces;
    Ipv4InterfaceContainer staInterfaces;
    std::vector<uint32_t> apNodeIndices;
    Ssid commonSsid;
    std::map<Mac48Address, uint32_t> apBssidToMeshIndex;
    std::map<uint32_t, uint32_t> nodeIdToStaIndex;
    std::vector<uint32_t> currentStaApIndex;
};

// LTE overlay configuration: single macro eNB + per-UE LTE interfaces
struct LteHybridConfig
{
    NodeContainer enbNodes;
    NetDeviceContainer enbDevices;
    NetDeviceContainer ueDevices;
    Ptr<LteHelper> lteHelper;
    Ptr<PointToPointEpcHelper> epcHelper;
    Ipv4InterfaceContainer ueIfaces;
    // LTE gateway on UE side (from EPC)
    Ipv4Address ueGateway;
    // Service destination IP (same as WiFi server IP)
    Ipv4Address serviceIp;
};

// NR overlay configuration: single macro gNB + per-UE NR interfaces
struct NrHybridConfig
{
    NodeContainer gnbNodes;
    NetDeviceContainer gnbDevices;
    NetDeviceContainer ueDevices;
    Ptr<NrHelper> nrHelper;
    Ptr<NrPointToPointEpcHelper> epcHelper;
    Ipv4InterfaceContainer ueIfaces;
    // NR gateway on UE side (from EPC)
    Ipv4Address ueGateway;
    // Service destination IP (same as WiFi server IP)
    Ipv4Address serviceIp;
};

namespace
{
struct StaRoamingContext
{
    MeshNetworkConfig* meshConfig{nullptr};
    HotspotConfig* hotspotConfig{nullptr};
    bool enabled{false};
};

StaRoamingContext g_staRoamingContext;

// Per-UE hybrid WiFi/LTE quality and path state
enum HybridPath
{
    USE_WIFI = 0,
    USE_LTE = 1,
    USE_NR = 2
};

struct HybridUeState
{
    HybridPath currentPath{USE_WIFI};
    double rssiAvgDbm{-1000.0};
    bool rssiInitialized{false};
    Time lastRssiSampleTime{Seconds(0.0)};
    std::vector<double> apRssiAvgDbm;
    std::vector<Time> apRssiLastUpdate;
    uint64_t txPackets{0};
    uint64_t rxPackets{0};
    std::deque<Time> txTimes;
    std::deque<Time> rxTimes;
    double pdrWindow{1.0};
    uint64_t lastTxSnapshot{0};
    uint64_t lastRxSnapshot{0};
    uint64_t lastWindowTxPackets{0};
    uint64_t lastWindowRxPackets{0};
    uint32_t wifiReturnGoodCount{0};
    Time lastPdrUpdate{Seconds(0.0)};
    Time lastSwitch{Seconds(0.0)};
    bool hasLastServiceRx{false};
    Time lastServiceRx{Seconds(0.0)};
    std::deque<uint64_t> pendingSwitchIds;
};

struct SwitchEventRecord
{
    uint64_t switchId{0};
    uint32_t staIndex{0};
    std::string from{"wifi"};
    std::string to{"wifi"};
    double triggerTimeS{0.0};
    double applyTimeS{0.0};
    double lastOkBeforeS{-1.0};
    double firstRxAfterSwitchS{-1.0};
    double serviceInterruptionMs{-1.0};
    std::string status{"pending"}; // pending | resolved | timeout | unresolved
    double rssiAvgDbm{-1000.0};
    Ipv4Address serviceIp{"0.0.0.0"};
    std::string triggerReason{"rssi"}; // rssi | pdr | rssi+pdr
};

struct HybridControllerContext
{
    bool enabled{false};
    // Indexed by STA index (0..numStaNodes-1)
    std::vector<HybridUeState> ueStates;
    // Map STA node ID -> STA index from HotspotConfig
    const std::map<uint32_t, uint32_t>* nodeIdToStaIndex{nullptr};
    // Full hotspot config (for AP/STA association and BSSIDs)
    const HotspotConfig* hotspotConfig{nullptr};
    // Simple thresholds for RSSI-based switching (can be tuned)
    double rssiThresholdDbm{-80.0};
    double pdrThreshold{0.9};
    double wifiReturnPdrThreshold{0.90};
    Time pdrWindowDuration{Seconds(1.0)};
    Time minSwitchInterval{Seconds(5.0)};
    Time switchTimeout{Seconds(5.0)};
    Time activityWindow{Seconds(2.0)};
    uint64_t minTxPacketsForSwitch{1};
    uint32_t wifiReturnGoodChecks{2};
    bool requireAssocForWifiReturn{false};
    bool kickAssocOnWifiReturn{true};
};

HybridControllerContext g_hybridContext;
std::ofstream g_rssiLog;
bool g_rssiLogOpen{false};
std::ofstream g_switchLog;
bool g_switchLogOpen{false};
std::ofstream g_pdrLog;
bool g_pdrLogOpen{false};
bool g_switchDebug{false};
Time g_apRssiFreshness{Seconds(1.0)};
uint64_t g_nextSwitchId{1};
std::vector<SwitchEventRecord> g_switchEvents;
std::map<uint64_t, std::size_t> g_switchEventIndex;

// Position of a STA at the current simulation time, formatted "x,y,z" for the
// RSSI log (feeds the trajectory plot and RSSI heatmap deliverables). Returns
// "nan,nan,nan" if the STA/mobility model can't be resolved.
static std::string
GetStaPositionCsv(uint32_t staIndex)
{
    if (g_hybridContext.hotspotConfig == nullptr ||
        staIndex >= g_hybridContext.hotspotConfig->staNodes.GetN())
    {
        return "nan,nan,nan";
    }
    Ptr<Node> staNode = g_hybridContext.hotspotConfig->staNodes.Get(staIndex);
    Ptr<MobilityModel> mob = staNode->GetObject<MobilityModel>();
    if (mob == nullptr)
    {
        return "nan,nan,nan";
    }
    Vector pos = mob->GetPosition();
    std::ostringstream oss;
    oss << pos.x << "," << pos.y << "," << pos.z;
    return oss.str();
}

static void
DumpStaRoutingTable(uint32_t staIndex, Ptr<Ipv4StaticRouting> rt, const std::string& label)
{
    if (!g_switchDebug || rt == nullptr)
    {
        return;
    }
    std::cout << Simulator::Now().GetSeconds() << "s [switch-debug] STA " << staIndex
              << " routing table (" << label << ")\n";
    Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(&std::cout);
    rt->PrintRoutingTable(stream, Time::S);
    std::cout << std::flush;
}

static bool
ExtractNodeIdFromContext(const std::string& context, uint32_t& nodeIdOut)
{
    // Context string example: "/NodeList/<id>/DeviceList/..."
    std::size_t nPos = context.find("NodeList/");
    if (nPos == std::string::npos)
    {
        return false;
    }
    nPos += std::string("NodeList/").size();
    std::size_t endPos = context.find('/', nPos);
    if (endPos == std::string::npos)
    {
        return false;
    }
    std::string idStr = context.substr(nPos, endPos - nPos);
    try
    {
        nodeIdOut = static_cast<uint32_t>(std::stoul(idStr));
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

static uint64_t
CreateSwitchEvent(uint32_t staIndex,
                  const std::string& fromPath,
                  const std::string& toPath,
                  Time triggerTime,
                  double rssiAvgDbm,
                  Ipv4Address serviceIp,
                  const std::string& triggerReason = "rssi")
{
    if (!g_hybridContext.enabled || staIndex >= g_hybridContext.ueStates.size())
    {
        return 0;
    }

    HybridUeState& st = g_hybridContext.ueStates[staIndex];

    // Fix head-of-line blocking: if earlier switches for this STA are still
    // pending when a new switch is decided, the controller changed the path
    // before service recovered on the previous decision. Retire those stale
    // pending events as "superseded" (a terminal state distinct from timeout)
    // so a single stalled event can no longer strand every switch behind it in
    // the FIFO. Their interruption is measured up to this new switch's apply
    // time (that is when the prior decision stopped being the active path).
    const double nowS = Simulator::Now().GetSeconds();
    while (!st.pendingSwitchIds.empty())
    {
        uint64_t staleId = st.pendingSwitchIds.front();
        st.pendingSwitchIds.pop_front();
        auto staleIt = g_switchEventIndex.find(staleId);
        if (staleIt == g_switchEventIndex.end())
        {
            continue;
        }
        SwitchEventRecord& stale = g_switchEvents[staleIt->second];
        if (stale.status != "pending")
        {
            continue;
        }
        const double baselineS = (stale.lastOkBeforeS >= 0.0) ? stale.lastOkBeforeS
                                                              : stale.applyTimeS;
        stale.serviceInterruptionMs = std::max(0.0, (nowS - baselineS) * 1000.0);
        stale.status = "superseded";
    }

    SwitchEventRecord ev;
    ev.switchId = g_nextSwitchId++;
    ev.staIndex = staIndex;
    ev.from = fromPath;
    ev.to = toPath;
    ev.triggerTimeS = triggerTime.GetSeconds();
    ev.applyTimeS = Simulator::Now().GetSeconds();
    ev.lastOkBeforeS = st.hasLastServiceRx ? st.lastServiceRx.GetSeconds() : -1.0;
    ev.rssiAvgDbm = rssiAvgDbm;
    ev.serviceIp = serviceIp;
    ev.status = "pending";
    ev.triggerReason = triggerReason;

    g_switchEventIndex[ev.switchId] = g_switchEvents.size();
    g_switchEvents.push_back(ev);
    st.pendingSwitchIds.push_back(ev.switchId);
    return ev.switchId;
}

static void
ResolvePendingSwitchOnRx(uint32_t staIndex, Time firstRxTime)
{
    if (!g_hybridContext.enabled || staIndex >= g_hybridContext.ueStates.size())
    {
        return;
    }
    HybridUeState& st = g_hybridContext.ueStates[staIndex];

    if (st.pendingSwitchIds.empty())
    {
        return;
    }

    uint64_t switchId = st.pendingSwitchIds.front();
    auto it = g_switchEventIndex.find(switchId);
    if (it == g_switchEventIndex.end())
    {
        st.pendingSwitchIds.pop_front();
        return;
    }

    SwitchEventRecord& ev = g_switchEvents[it->second];
    ev.firstRxAfterSwitchS = firstRxTime.GetSeconds();
    if (ev.lastOkBeforeS >= 0.0)
    {
        ev.serviceInterruptionMs = (ev.firstRxAfterSwitchS - ev.lastOkBeforeS) * 1000.0;
    }
    else
    {
        // If there was no prior successful RX snapshot, use switch apply time
        // as a fallback baseline so early/cold-start switches still get a
        // meaningful recovery delay value.
        ev.serviceInterruptionMs = (ev.firstRxAfterSwitchS - ev.applyTimeS) * 1000.0;
    }
    ev.status = "resolved";
    st.pendingSwitchIds.pop_front();
}

void
HandleServiceSinkRx(uint32_t staIndex, Ptr<const Packet> packet, const Address& from)
{
    (void)packet;
    (void)from;

    if (!g_hybridContext.enabled || staIndex >= g_hybridContext.ueStates.size())
    {
        return;
    }

    HybridUeState& st = g_hybridContext.ueStates[staIndex];
    st.rxPackets++;
    st.rxTimes.push_back(Simulator::Now());
    st.lastServiceRx = Simulator::Now();
    st.hasLastServiceRx = true;
    ResolvePendingSwitchOnRx(staIndex, Simulator::Now());
}

// Control-flow (UDP) liveness for switch resolution.
//
// The Sensor TCP flow is a poor liveness probe after a path switch: TCP enters
// RTO backoff / slow-start when in-flight segments are lost during the route
// rewrite, so a fully-working new path can still take seconds to produce a TCP
// sink RX — inflating serviceInterruptionMs and causing false "timeout"s.
//
// The Control flow is continuous small UDP (every 50 ms) and recovers on the
// new path almost immediately, so its server-side RX is the accurate signal for
// "service has resumed after the switch". This handler updates the liveness
// baseline and resolves pending switches, but deliberately does NOT touch the
// PDR sliding window (rxTimes/rxPackets) — UDP is intentionally excluded from
// PDR to avoid spurious PDR-based switches.
static void
HandleControlLivenessRx(uint32_t staIndex, Ptr<const Packet> packet)
{
    (void)packet;
    if (!g_hybridContext.enabled || staIndex >= g_hybridContext.ueStates.size())
    {
        return;
    }

    HybridUeState& st = g_hybridContext.ueStates[staIndex];
    st.lastServiceRx = Simulator::Now();
    st.hasLastServiceRx = true;
    ResolvePendingSwitchOnRx(staIndex, Simulator::Now());
}

static void
HandleServiceTx(uint32_t staIndex, Ptr<const Packet> packet)
{
    (void)packet;
    if (!g_hybridContext.enabled || staIndex >= g_hybridContext.ueStates.size())
    {
        return;
    }

    HybridUeState& st = g_hybridContext.ueStates[staIndex];
    st.txPackets++;
    st.txTimes.push_back(Simulator::Now());
}

static inline void
PruneWindow(std::deque<Time>& q, Time cutoff)
{
    while (!q.empty() && q.front() < cutoff)
    {
        q.pop_front();
    }
}

static void
UpdatePerStaPdr(Time interval)
{
    if (!g_hybridContext.enabled)
    {
        return;
    }

    Time now = Simulator::Now();
    Time cutoff = now - g_hybridContext.pdrWindowDuration;
    for (uint32_t i = 0; i < g_hybridContext.ueStates.size(); ++i)
    {
        HybridUeState& st = g_hybridContext.ueStates[i];
        PruneWindow(st.txTimes, cutoff);
        PruneWindow(st.rxTimes, cutoff);

        uint64_t dTx = static_cast<uint64_t>(st.txTimes.size());
        uint64_t dRx = static_cast<uint64_t>(st.rxTimes.size());

        double pdr = (dTx > 0) ? static_cast<double>(dRx) / static_cast<double>(dTx) : 1.0;
        st.pdrWindow = pdr;
        st.lastWindowTxPackets = dTx;
        st.lastWindowRxPackets = dRx;
        st.lastPdrUpdate = now;

        if (g_pdrLogOpen)
        {
            std::string pathStr = "wifi";
            if (st.currentPath == USE_LTE)
            {
                pathStr = "lte";
            }
            else if (st.currentPath == USE_NR)
            {
                pathStr = "nr";
            }
            g_pdrLog << now.GetSeconds() << ","
                     << i << ","
                     << dTx << ","
                     << dRx << ","
                     << pdr << ","
                     << pathStr << "\n";
        }
    }

    Simulator::Schedule(interval, &UpdatePerStaPdr, interval);
}

static void
CheckPendingSwitchTimeouts(Time interval)
{
    if (!g_hybridContext.enabled)
    {
        return;
    }

    const double nowS = Simulator::Now().GetSeconds();
    const double timeoutS = g_hybridContext.switchTimeout.GetSeconds();
    for (auto& st : g_hybridContext.ueStates)
    {
        while (!st.pendingSwitchIds.empty())
        {
            uint64_t switchId = st.pendingSwitchIds.front();
            auto it = g_switchEventIndex.find(switchId);
            if (it == g_switchEventIndex.end())
            {
                st.pendingSwitchIds.pop_front();
                continue;
            }

            SwitchEventRecord& ev = g_switchEvents[it->second];
            if (ev.status != "pending")
            {
                st.pendingSwitchIds.pop_front();
                continue;
            }
            if ((nowS - ev.applyTimeS) < timeoutS)
            {
                break;
            }

            const double baselineS = (ev.lastOkBeforeS >= 0.0) ? ev.lastOkBeforeS : ev.applyTimeS;
            ev.serviceInterruptionMs = std::max(0.0, (nowS - baselineS) * 1000.0);
            ev.status = "timeout";
            st.pendingSwitchIds.pop_front();
        }
    }

    Simulator::Schedule(interval, &CheckPendingSwitchTimeouts, interval);
}

void
RemoveExistingHostRoute(Ptr<Ipv4StaticRouting> routing, const Ipv4Address& destination)
{
    // Remove all host routes to this destination so switching leaves exactly
    // one active host route (the one added by the current path decision).
    for (int32_t idx = static_cast<int32_t>(routing->GetNRoutes()) - 1; idx >= 0; --idx)
    {
        Ipv4RoutingTableEntry entry = routing->GetRoute(static_cast<uint32_t>(idx));
        if (entry.IsHost() && entry.GetDest() == destination)
        {
            routing->RemoveRoute(static_cast<uint32_t>(idx));
        }
    }
}

// Remove host routes to `destination` that go through any interface OTHER than `keepIf`.
// Used for make-before-break: the new route is installed first (on keepIf), then stale
// routes on the old interface are cleaned up without ever leaving the destination unreachable.
void
RemoveHostRoutesExcept(Ptr<Ipv4StaticRouting> routing,
                       const Ipv4Address& destination,
                       uint32_t keepIf)
{
    for (int32_t idx = static_cast<int32_t>(routing->GetNRoutes()) - 1; idx >= 0; --idx)
    {
        Ipv4RoutingTableEntry entry = routing->GetRoute(static_cast<uint32_t>(idx));
        if (entry.IsHost() && entry.GetDest() == destination && entry.GetInterface() != keepIf)
        {
            routing->RemoveRoute(static_cast<uint32_t>(idx));
        }
    }
}

void
RemoveExistingDefaultRoute(Ptr<Ipv4StaticRouting> routing)
{
    for (uint32_t idx = 0; idx < routing->GetNRoutes(); ++idx)
    {
        Ipv4RoutingTableEntry entry = routing->GetRoute(idx);
        if (entry.IsDefault())
        {
            routing->RemoveRoute(idx);
            return;
        }
    }
}

void
UpdateStaRouting(uint32_t staIndex, uint32_t apNodeIndex)
{
    if (!g_staRoamingContext.enabled || g_staRoamingContext.meshConfig == nullptr ||
        g_staRoamingContext.hotspotConfig == nullptr)
    {
        return;
    }

    MeshNetworkConfig& meshConfig = *g_staRoamingContext.meshConfig;
    HotspotConfig& hotspotConfig = *g_staRoamingContext.hotspotConfig;

    NS_ABORT_IF(staIndex >= hotspotConfig.staNodes.GetN());
    NS_ABORT_IF(apNodeIndex >= meshConfig.meshNodes.GetN());

    Ipv4Address staIp = hotspotConfig.staInterfaces.GetAddress(staIndex);
    Ipv4Address apMeshIp = meshConfig.meshInterfaces.GetAddress(apNodeIndex);
    Ipv4Address apHotspotIp = hotspotConfig.apInterfaces.GetAddress(apNodeIndex);

    Ipv4StaticRoutingHelper staticRouting;

    // Update gateway host route
    Ptr<Node> gatewayNode = meshConfig.meshNodes.Get(0);
    Ptr<Ipv4> gatewayIpv4 = gatewayNode->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> gatewayRouting = staticRouting.GetStaticRouting(gatewayIpv4);
    Ptr<NetDevice> gatewayMeshDevice = meshConfig.meshDevices.Get(0);
    uint32_t gatewayMeshInterface = gatewayIpv4->GetInterfaceForDevice(gatewayMeshDevice);
    RemoveExistingHostRoute(gatewayRouting, staIp);
    if (apNodeIndex == 0)
    {
        // STA is directly on the gateway's own AP. The 192.168.1.0/24 subnet is
        // directly connected via the gateway's AP interface, so no explicit host
        // route is needed — NS-3 subnet routing delivers to the STA directly.
        // Adding an incorrect host route here would override the subnet route and
        // cause the packet to loop back to the gateway itself.
    }
    else
    {
        gatewayRouting->AddHostRouteTo(staIp, apMeshIp, gatewayMeshInterface);
    }

    // Update STA default route
    Ptr<Node> staNode = hotspotConfig.staNodes.Get(staIndex);
    Ptr<Ipv4> staIpv4 = staNode->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> staRouting = staticRouting.GetStaticRouting(staIpv4);
    Ptr<NetDevice> staDevice = hotspotConfig.staDevices.Get(staIndex);
    uint32_t staInterface = staIpv4->GetInterfaceForDevice(staDevice);
    RemoveExistingDefaultRoute(staRouting);
    staRouting->SetDefaultRoute(apHotspotIp, staInterface);

    hotspotConfig.currentStaApIndex[staIndex] = apNodeIndex;

    std::cout << Simulator::Now().GetSeconds() << "s STA " << staIndex
              << " host route via mesh node " << apNodeIndex << " (STA IP " << staIp
              << ", AP mesh IP " << apMeshIp << ", AP hotspot IP " << apHotspotIp << ")"
              << std::endl;
}

void
HandleStaAssociation(uint32_t nodeId, Mac48Address apAddress)
{
    if (!g_staRoamingContext.enabled || g_staRoamingContext.hotspotConfig == nullptr)
    {
        return;
    }

    HotspotConfig& hotspotConfig = *g_staRoamingContext.hotspotConfig;
    auto staIt = hotspotConfig.nodeIdToStaIndex.find(nodeId);
    if (staIt == hotspotConfig.nodeIdToStaIndex.end())
    {
        return;
    }
    uint32_t staIndex = staIt->second;

    auto apIt = hotspotConfig.apBssidToMeshIndex.find(apAddress);
    if (apIt == hotspotConfig.apBssidToMeshIndex.end())
    {
        NS_LOG_WARN("STA " << staIndex << " associated with unknown AP BSSID " << apAddress);
        return;
    }
    uint32_t apNodeIndex = apIt->second;
    UpdateStaRouting(staIndex, apNodeIndex);
}

void
HandleStaDeAssociation(uint32_t nodeId, Mac48Address apAddress)
{
    if (!g_staRoamingContext.enabled || g_staRoamingContext.hotspotConfig == nullptr)
    {
        return;
    }

    HotspotConfig& hotspotConfig = *g_staRoamingContext.hotspotConfig;
    auto staIt = hotspotConfig.nodeIdToStaIndex.find(nodeId);
    if (staIt == hotspotConfig.nodeIdToStaIndex.end())
    {
        return;
    }
    uint32_t staIndex = staIt->second;
    hotspotConfig.currentStaApIndex[staIndex] = std::numeric_limits<uint32_t>::max();

    std::cout << Simulator::Now().GetSeconds() << "s STA " << staIndex
              << " disassociated from AP " << apAddress << std::endl;
}

void
EnableStaRoamingTracing(MeshNetworkConfig& meshConfig, HotspotConfig& hotspotConfig, bool enableHotspot)
{
    if (!enableHotspot)
    {
        return;
    }

    g_staRoamingContext.meshConfig = &meshConfig;
    g_staRoamingContext.hotspotConfig = &hotspotConfig;
    g_staRoamingContext.enabled = true;

    std::cout << "STA roaming tracing enabled (shared SSID: " << hotspotConfig.commonSsid.PeekString()
              << ")" << std::endl;

    uint32_t tracedCount = 0;
    for (uint32_t i = 0; i < hotspotConfig.staDevices.GetN(); ++i)
    {
        Ptr<WifiNetDevice> staDevice = DynamicCast<WifiNetDevice>(hotspotConfig.staDevices.Get(i));
        if (!staDevice)
        {
            continue;
        }
        Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac>(staDevice->GetMac());
        if (!staMac)
        {
            continue;
        }
        uint32_t nodeId = hotspotConfig.staNodes.Get(i)->GetId();
        staMac->TraceConnectWithoutContext("Assoc",
                                           MakeBoundCallback(&HandleStaAssociation, nodeId));
        staMac->TraceConnectWithoutContext("DeAssoc",
                                           MakeBoundCallback(&HandleStaDeAssociation, nodeId));
        std::cout << "  STA node " << nodeId << " trace hooks attached" << std::endl;
        ++tracedCount;
    }

    std::cout << "  Registered association callbacks for " << tracedCount << " STA MAC(s)"
              << std::endl;
}

} // namespace

std::vector<Vector> GetFourNodeMeshPositions(double meshApHeight)
{
    return {
        Vector(100.0, 100.0, meshApHeight),
        Vector(300.0, 100.0, meshApHeight),
        Vector(300.0, 300.0, meshApHeight),
        Vector(100.0, 300.0, meshApHeight)
    };
}

MeshNetworkConfig SetupMeshNetwork(uint32_t nNodes, uint32_t gridWidth, double nodeSpacing, double meshApHeight, const MeshAPDeviceConfig& deviceCfg)
{
    NS_LOG_FUNCTION("Setting up mesh network with " << nNodes << " nodes at height " << meshApHeight << "m");
    NS_LOG_INFO("Using device config: " << deviceCfg.name);
    
    MeshNetworkConfig config;
    
    // Create mesh nodes
    config.meshNodes.Create(nNodes);
    
    // Set up Wi-Fi Mesh with configured standard
    WifiMacHelper wifiMac;
    WifiHelper wifi;
    
    // Set WiFi standard from device configuration
    wifi.SetStandard(deviceCfg.wifiStandard);
    
    NS_LOG_INFO("WiFi Standard: " << deviceCfg.wifiStandard << ", Data Mode: " << deviceCfg.dataMode);

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    
    // Use HybridBuildingsPropagationLossModel with explicit parameters to align with LTE/NR
    wifiChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
                                   "Frequency",
                                   DoubleValue(2.4e9));

    config.wifiPhy.SetChannel(wifiChannel.Create());
    
    // Apply device-specific PHY parameters
    config.wifiPhy.Set("TxPowerStart", DoubleValue(deviceCfg.txPowerStart));
    config.wifiPhy.Set("TxPowerEnd", DoubleValue(deviceCfg.txPowerEnd));
    config.wifiPhy.Set("RxSensitivity", DoubleValue(deviceCfg.rxSensitivity));
    config.wifiPhy.Set("RxGain", DoubleValue(deviceCfg.rxGain));
    config.wifiPhy.Set("TxGain", DoubleValue(deviceCfg.txGain));
    
    NS_LOG_INFO("TX Power: " << deviceCfg.txPowerStart << " dBm, RX Sensitivity: " << deviceCfg.rxSensitivity << " dBm");
    NS_LOG_INFO("Antenna Gains - RX: " << deviceCfg.rxGain << " dB, TX: " << deviceCfg.txGain << " dB");

    // Enable ASCII tracing *before* installing devices
    // AsciiTraceHelper ascii;
    // config.wifiPhy.EnableAsciiAll(ascii.CreateFileStream("wifi_test_research/six_node_layout/wifi-test-2-adhoc-grid-six.tr"));

    MeshHelper mesh;
    mesh = MeshHelper::Default();
    mesh.SetStackInstaller("ns3::Dot11sStack");
    mesh.SetSpreadInterfaceChannels(MeshHelper::SPREAD_CHANNELS);
    mesh.SetMacType("RandomStart", TimeValue(Seconds(0.1)));
    mesh.SetNumberOfInterfaces(deviceCfg.numInterfaces);  // Use device config

    config.meshDevices = mesh.Install(config.wifiPhy, config.meshNodes);
    
    NS_LOG_INFO("Mesh installed with " << deviceCfg.numInterfaces << " interface(s)");

    // Install Internet stack
    InternetStackHelper internetStack;
    internetStack.Install(config.meshNodes);

    // Assign IP addresses to mesh network
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    config.meshInterfaces = address.Assign(config.meshDevices);

    // Set up mobility model (custom four-node layout)
    std::vector<Vector> meshPositions = GetFourNodeMeshPositions(meshApHeight);

    if (nNodes > meshPositions.size())
    {
        NS_LOG_WARN("Requested " << nNodes << " mesh nodes, but four-node layout only defines "
                                 << meshPositions.size() << " positions. Extra nodes will use grid fallback.");
    }

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nNodes; ++i)
    {
        Vector pos;
        if (i < meshPositions.size())
        {
            pos = meshPositions[i];
        }
        else
        {
            uint32_t fallbackWidth = gridWidth > 0 ? gridWidth : static_cast<uint32_t>(std::ceil(std::sqrt(nNodes)));
            double x = (i % fallbackWidth) * nodeSpacing;
            double y = (i / fallbackWidth) * nodeSpacing;
            pos = Vector(x, y, meshApHeight);
        }
        positionAlloc->Add(pos);
        NS_LOG_INFO("Mesh node " << i << " position set to (" << pos.x << ", " << pos.y << ", " << pos.z << ")");
    }

    MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(config.meshNodes);
    
    // Fixed building layout to match 5G/LTE playfield scenarios (400m x 400m field)
    BuildingContainer buildings;
    Ptr<Building> leftBelow = CreateObject<Building>();
    leftBelow->SetBoundaries(Box(0.0, 60.0, 96.0, 104.0, 0.0, 10.0));
    leftBelow->SetBuildingType(Building::Residential);
    leftBelow->SetExtWallsType(Building::ConcreteWithWindows);
    buildings.Add(leftBelow);
    Ptr<Building> rightBelow = CreateObject<Building>();
    rightBelow->SetBoundaries(Box(340.0, 400.0, 96.0, 104.0, 0.0, 10.0));
    rightBelow->SetBuildingType(Building::Residential);
    rightBelow->SetExtWallsType(Building::ConcreteWithWindows);
    buildings.Add(rightBelow);
    Ptr<Building> leftAbove = CreateObject<Building>();
    leftAbove->SetBoundaries(Box(0.0, 60.0, 296.0, 304.0, 0.0, 10.0));
    leftAbove->SetBuildingType(Building::Residential);
    leftAbove->SetExtWallsType(Building::ConcreteWithWindows);
    buildings.Add(leftAbove);
    Ptr<Building> rightAbove = CreateObject<Building>();
    rightAbove->SetBoundaries(Box(340.0, 400.0, 296.0, 304.0, 0.0, 10.0));
    rightAbove->SetBuildingType(Building::Residential);
    rightAbove->SetExtWallsType(Building::ConcreteWithWindows);
    buildings.Add(rightAbove);
    Ptr<Building> cluster250a = CreateObject<Building>();
    cluster250a->SetBoundaries(Box(80.0, 140.0, 320.0, 328.0, 0.0, 15.0));
    cluster250a->SetBuildingType(Building::Office);
    cluster250a->SetExtWallsType(Building::ConcreteWithWindows);
    buildings.Add(cluster250a);
    Ptr<Building> cluster250b = CreateObject<Building>();
    cluster250b->SetBoundaries(Box(170.0, 250.0, 300.0, 308.0, 0.0, 12.0));
    cluster250b->SetBuildingType(Building::Office);
    cluster250b->SetExtWallsType(Building::ConcreteWithWindows);
    buildings.Add(cluster250b);
    Ptr<Building> cluster50 = CreateObject<Building>();
    cluster50->SetBoundaries(Box(255.0, 335.0, 20.0, 28.0, 0.0, 18.0));
    cluster50->SetBuildingType(Building::Commercial);
    cluster50->SetExtWallsType(Building::ConcreteWithWindows);
    buildings.Add(cluster50);

    BuildingsHelper::Install(config.meshNodes);

    NS_LOG_INFO("Mesh network setup complete with fixed building layout (400m x 400m x 30m)");
    NS_LOG_INFO("  Node spacing: " << nodeSpacing << "m");
    return config;
}

InternetConfig SetupInternetInfrastructure(NodeContainer meshNodes, double nodeSpacing, double meshApHeight)
{
    NS_LOG_FUNCTION("Setting up internet infrastructure");
    
    InternetConfig config;
    
    // Create internet infrastructure nodes
    config.internetNodes.Create(2);  // ISP router + Internet server
    
    // Install internet stack
    InternetStackHelper internetStack;
    internetStack.Install(config.internetNodes);
    
    // Create Ethernet (CSMA) backhaul link from node 0 to ISP router
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("1Gbps"));
    csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));

    NodeContainer backhaulLink;
    backhaulLink.Add(meshNodes.Get(0));      // Mesh gateway (node 0)
    backhaulLink.Add(config.internetNodes.Get(0));  // ISP router
    config.backhaulDevices = csma.Install(backhaulLink);

    // Create internet backbone (ISP router to internet server)
    NodeContainer internetBackbone;
    internetBackbone.Add(config.internetNodes.Get(0));  // ISP router
    internetBackbone.Add(config.internetNodes.Get(1));  // Internet server
    config.backboneDevices = csma.Install(internetBackbone);
    
    // Assign IP addresses to backhaul link
    Ipv4AddressHelper backhaulAddress;
    backhaulAddress.SetBase("192.168.100.0", "255.255.255.0");
    config.backhaulInterfaces = backhaulAddress.Assign(config.backhaulDevices);

    // Assign IP addresses to internet backbone (simulating public IPs)
    Ipv4AddressHelper internetAddress;
    internetAddress.SetBase("8.8.8.0", "255.255.255.0");
    config.internetInterfaces = internetAddress.Assign(config.backboneDevices);
    
    // Set up mobility for internet nodes (stationary)
    MobilityHelper internetMobility;
    internetMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    internetMobility.Install(config.internetNodes);
    
    Ptr<ConstantPositionMobilityModel> posRouter = 
        config.internetNodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
    posRouter->SetPosition(Vector(-400.0, nodeSpacing, meshApHeight));  // Same height as mesh APs
    
    Ptr<ConstantPositionMobilityModel> posServer = 
        config.internetNodes.Get(1)->GetObject<ConstantPositionMobilityModel>();
    posServer->SetPosition(Vector(-700.0, nodeSpacing, meshApHeight));  // Same height as mesh APs
    
    // Aggregate building information to internet nodes
    BuildingsHelper::Install(config.internetNodes);
    
    NS_LOG_INFO("Internet infrastructure setup complete");
    return config;
}

// ---------------------------------------------------------------------------
// Phase 2 Item 1: Waypoint + dwell-time mobility.
// Builds a fixed-shape route for one of three construction-robot archetypes
// (see NS3_Simulation_Enhancement_Plan_EN.docx section 2.2) so RSSI variation
// reflects structured movement instead of GaussMarkovMobilityModel's random
// walk. The route is repeated/extended until it covers simTime, since
// WaypointMobilityModel freezes the node at its last waypoint's position.
// ---------------------------------------------------------------------------
std::vector<Waypoint> BuildRobotWaypointRoute(const std::string& robotType,
                                               double speed,
                                               double simTime,
                                               double fieldSize,
                                               double zValue,
                                               uint32_t staIndex,
                                               uint32_t numStaNodes,
                                               Ptr<UniformRandomVariable> dwellRng)
{
    std::vector<Waypoint> route;
    double t = 0.0;
    const double margin = 30.0; // pad past simTime so the last leg isn't truncated
    const double effSpeed = speed > 0.0 ? speed : 1.0;

    // z defaults to the STA's assigned height, but callers can override it
    // per-waypoint to give each robot type genuine 3D movement instead of a
    // flat plane. WaypointMobilityModel interpolates linearly in x/y/z
    // between waypoints, so a z override on just one endpoint of a leg
    // produces a smooth climb/descent over that leg.
    auto at = [zValue](double x, double y, double zOverride = -1.0) {
        return Vector(x, y, zOverride >= 0.0 ? zOverride : zValue);
    };

    auto addLeg = [&](Vector from, Vector to, double dwellAtTo) {
        double dist = CalculateDistance(from, to);
        t += dist / effSpeed;
        route.emplace_back(Seconds(t), to);
        if (dwellAtTo > 0.0)
        {
            t += dwellAtTo;
            route.emplace_back(Seconds(t), to);
        }
    };

    if (robotType == "transport")
    {
        // Logistics robot: back-and-forth along a corridor between the
        // building clusters, decelerating over the last 15m approaching each
        // end wall (matches "path movement" in the enhancement plan). Height
        // rises slightly (loading-dock ramp) over that last 15m and settles
        // back to ground level mid-corridor, giving genuine (if modest) 3D
        // movement for a ground vehicle.
        double corridorY = std::min(fieldSize - 20.0, 60.0 + 40.0 * (staIndex % 7));
        double xMin = 60.0;
        double xMax = fieldSize - 60.0;
        const double dockZ = zValue + 1.5;
        Vector a = at(xMin, corridorY, dockZ);
        Vector b = at(xMax, corridorY, dockZ);
        route.emplace_back(Seconds(0.0), a);
        Vector prev = a;
        bool towardB = true;
        while (t < simTime + margin)
        {
            Vector target = towardB ? b : a;
            double dirSign = towardB ? -1.0 : 1.0;
            Vector decelPoint = at(target.x + dirSign * 15.0, target.y, zValue);
            addLeg(prev, decelPoint, 0.0);
            double slowSpeed = std::max(0.2, effSpeed * 0.5);
            t += 15.0 / slowSpeed;
            route.emplace_back(Seconds(t), target);
            prev = target;
            towardB = !towardB;
        }
    }
    else if (robotType == "work")
    {
        // Welding/finishing robot: long dwell (30-120s) at fixed work sites
        // placed beside real building edges, reproducing the sharp RSSI-drop
        // "obstacle approach" zones described in the enhancement plan. One
        // site per building (7 total) so STAs don't stack on identical
        // coordinates until numStaNodes exceeds 7. Each site's height is set
        // to roughly the middle floor of its building (heights match the
        // Box() z-extents in SetupMeshNetwork), so travel between sites
        // reproduces the "elevator shaft" elevation jumps the enhancement
        // plan describes, not just an x/y move.
        std::vector<Vector> workSites = {
            at(30.0, 90.0, zValue + 5.0),    // beside Residential leftBelow (10m tall)
            at(370.0, 90.0, zValue + 5.0),   // beside Residential rightBelow (10m tall)
            at(30.0, 290.0, zValue + 5.0),   // beside Residential leftAbove (10m tall)
            at(370.0, 290.0, zValue + 5.0),  // beside Residential rightAbove (10m tall)
            at(115.0, 312.0, zValue + 7.5),  // beside Office cluster250a (15m tall)
            at(210.0, 292.0, zValue + 6.0),  // beside Office cluster250b (12m tall)
            at(295.0, 36.0, zValue + 9.0),   // beside Commercial cluster50 (18m tall)
        };
        Vector prev = workSites[staIndex % workSites.size()];
        route.emplace_back(Seconds(0.0), prev);
        double dwell0 = dwellRng->GetValue(30.0, 120.0);
        t += dwell0;
        route.emplace_back(Seconds(t), prev);
        uint32_t idx = 1;
        while (t < simTime + margin)
        {
            Vector next = workSites[(staIndex + idx) % workSites.size()];
            double dwell = dwellRng->GetValue(30.0, 120.0);
            addLeg(prev, next, dwell);
            prev = next;
            ++idx;
        }
    }
    else // "patrol" (default)
    {
        // Surveillance robot: repeated loop over 4 waypoints, dwelling 5-30s
        // at each corner. Box centers are placed on a grid sized to
        // numStaNodes so routes spread across the full field instead of
        // clustering in one corner regardless of how many STAs are used.
        // Altitude varies per corner (survey drone climbing for a wider
        // view then descending), giving genuine 3D movement instead of a
        // flat patrol plane, capped so it stays under the 30m field ceiling.
        const double maxZ = 28.0;
        const std::array<double, 4> cornerZOffsets = {0.0, 5.0, 9.0, 5.0};
        const double half = 50.0;
        const double usableMin = 60.0 + half;
        const double usableMax = fieldSize - 60.0 - half;
        uint32_t cols = std::max<uint32_t>(1, static_cast<uint32_t>(std::ceil(std::sqrt(
            static_cast<double>(std::max<uint32_t>(1, numStaNodes))))));
        uint32_t rows = std::max<uint32_t>(1, (std::max<uint32_t>(1, numStaNodes) + cols - 1) / cols);
        uint32_t col = staIndex % cols;
        uint32_t row = (staIndex / cols) % rows;
        double colSpacing = cols > 1 ? (usableMax - usableMin) / (cols - 1) : 0.0;
        double rowSpacing = rows > 1 ? (usableMax - usableMin) / (rows - 1) : 0.0;
        double cx = cols > 1 ? usableMin + col * colSpacing : (usableMin + usableMax) / 2.0;
        double cy = rows > 1 ? usableMin + row * rowSpacing : (usableMin + usableMax) / 2.0;
        std::vector<Vector> corners = {
            at(cx - half, cy - half, std::min(maxZ, zValue + cornerZOffsets[0])),
            at(cx + half, cy - half, std::min(maxZ, zValue + cornerZOffsets[1])),
            at(cx + half, cy + half, std::min(maxZ, zValue + cornerZOffsets[2])),
            at(cx - half, cy + half, std::min(maxZ, zValue + cornerZOffsets[3])),
        };
        Vector prev = corners[0];
        route.emplace_back(Seconds(0.0), prev);
        uint32_t idx = 1;
        while (t < simTime + margin)
        {
            Vector next = corners[idx % corners.size()];
            double dwell = dwellRng->GetValue(5.0, 30.0);
            addLeg(prev, next, dwell);
            prev = next;
            ++idx;
        }
    }

    return route;
}

HotspotConfig SetupHotspotInfrastructure(NodeContainer meshNodes,
                                          uint32_t /*primaryApNodeIndex*/,
                                          uint32_t numStaNodes,
                                          double nodeSpacing,
                                          double staHeight,
                                          double staSpeedMin,
                                          double staSpeedMax,
                                          const MeshAPDeviceConfig& deviceCfg,
                                          const std::string& hotspotBand,
                                          double simTime,
                                          const std::string& mobilityModel,
                                          const std::string& robotType)
{
    NS_LOG_FUNCTION("Setting up hotspot infrastructure with " << numStaNodes << " STA clients");
    NS_LOG_INFO("Using hotspot TX power: " << deviceCfg.hotspotTxPower << " dBm from device config");
    NS_LOG_INFO("STA mobility speed range: [" << staSpeedMin << ", " << staSpeedMax << "] m/s");

    HotspotConfig config;
    config.commonSsid = Ssid("MeshHotspot");

    uint32_t meshCount = meshNodes.GetN();
    if (numStaNodes < meshCount)
    {
        NS_LOG_WARN("Requested " << numStaNodes << " STAs but there are " << meshCount
                     << " mesh nodes; some APs will not have dedicated clients.");
    }

    // Create STA nodes and install Internet stack
    config.staNodes.Create(numStaNodes);
    InternetStackHelper internetStack;
    internetStack.Install(config.staNodes);

    // Wi-Fi helpers for hotspot radios
    std::string hotspotBandLower = hotspotBand;
    std::transform(hotspotBandLower.begin(),
                   hotspotBandLower.end(),
                   hotspotBandLower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    bool use5gHotspot = hotspotBandLower != "2g";
    double hotspotFrequencyHz = use5gHotspot ? 5.18e9 : 2.437e9;
    WifiHelper hotspotWifi;
    if (use5gHotspot)
    {
        hotspotWifi.SetStandard(WIFI_STANDARD_80211ac);
        NS_LOG_INFO("  Hotspot band: 5 GHz (802.11ac)");
    }
    else
    {
        hotspotWifi.SetStandard(WIFI_STANDARD_80211n);
        NS_LOG_INFO("  Hotspot band: 2.4 GHz (802.11n)");
    }

    YansWifiChannelHelper hotspotChannel;
    hotspotChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    hotspotChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel",
                                      "Frequency",
                                      DoubleValue(hotspotFrequencyHz));

    Ptr<YansWifiChannel> sharedHotspotChannel = hotspotChannel.Create();

    // AP PHY: high-gain, higher power
    YansWifiPhyHelper apPhy;
    apPhy.SetChannel(sharedHotspotChannel);
    apPhy.Set("TxPowerStart", DoubleValue(deviceCfg.hotspotTxPower));
    apPhy.Set("TxPowerEnd", DoubleValue(deviceCfg.hotspotTxPower));
    apPhy.Set("TxGain", DoubleValue(deviceCfg.txGain));
    apPhy.Set("RxGain", DoubleValue(deviceCfg.rxGain));
    apPhy.Set("RxSensitivity", DoubleValue(deviceCfg.rxSensitivity));
    apPhy.Set("RxNoiseFigure", DoubleValue(5.0));

    // STA PHY: handheld client budget
    YansWifiPhyHelper staPhy;
    staPhy.SetChannel(sharedHotspotChannel);
    staPhy.Set("TxPowerStart", DoubleValue(15.0));
    staPhy.Set("TxPowerEnd", DoubleValue(15.0));
    staPhy.Set("TxGain", DoubleValue(1.0));
    staPhy.Set("RxGain", DoubleValue(1.0));
    staPhy.Set("RxSensitivity", DoubleValue(-90.0));
    staPhy.Set("RxNoiseFigure", DoubleValue(7.0));

    // Optional ASCII tracing
    // AsciiTraceHelper ascii;
    // apPhy.EnableAsciiAll(ascii.CreateFileStream("wifi_test_research/six_node_layout/wifi-test-2-ap-six.tr"));
    // staPhy.EnableAsciiAll(ascii.CreateFileStream("wifi_test_research/six_node_layout/wifi-test-2-sta-six.tr"));

    // Install AP devices on every mesh node (shared SSID for roaming)
    for (uint32_t i = 0; i < meshCount; ++i)
    {
        config.apNodeIndices.push_back(i);
        WifiMacHelper apMac;
        apMac.SetType("ns3::ApWifiMac",
                      "Ssid", SsidValue(config.commonSsid),
                      "BeaconGeneration", BooleanValue(true),
                      "BeaconInterval", TimeValue(MicroSeconds(102400)));
        NetDeviceContainer apDev = hotspotWifi.Install(apPhy, apMac, meshNodes.Get(i));
        config.apDevices.Add(apDev);

        Ptr<WifiNetDevice> apWifiDev = DynamicCast<WifiNetDevice>(apDev.Get(0));
        if (apWifiDev)
        {
            Mac48Address bssid = apWifiDev->GetMac()->GetBssid(0);
            config.apBssidToMeshIndex[bssid] = i;
        }
    }

    // Install STA devices that can freely roam
    for (uint32_t i = 0; i < numStaNodes; ++i)
    {
        WifiMacHelper staMac;
        staMac.SetType("ns3::StaWifiMac",
                       "Ssid", SsidValue(config.commonSsid),
                       "ActiveProbing", BooleanValue(true));
        NetDeviceContainer staDev = hotspotWifi.Install(staPhy, staMac, config.staNodes.Get(i));
        config.staDevices.Add(staDev);

        Ptr<Node> staNode = config.staNodes.Get(i);
        config.nodeIdToStaIndex[staNode->GetId()] = i;
        config.currentStaApIndex.push_back(std::numeric_limits<uint32_t>::max());
    }

    // Assign unique IP ranges
    Ipv4AddressHelper staAddress;
    staAddress.SetBase("192.168.1.0", "255.255.255.0", "0.0.0.1");
    config.staInterfaces = staAddress.Assign(config.staDevices);

    Ipv4AddressHelper apAddress;
    apAddress.SetBase("192.168.1.0", "255.255.255.0", "0.0.0.101");
    config.apInterfaces = apAddress.Assign(config.apDevices);

    // Configure STA initial positions using uniform distribution
    Ptr<UniformRandomVariable> xDist = CreateObject<UniformRandomVariable>();
    xDist->SetAttribute("Min", DoubleValue(0.0));
    xDist->SetAttribute("Max", DoubleValue(400.0));

    Ptr<UniformRandomVariable> yDist = CreateObject<UniformRandomVariable>();
    yDist->SetAttribute("Min", DoubleValue(0.0));
    yDist->SetAttribute("Max", DoubleValue(400.0));

    Ptr<UniformRandomVariable> zDist = CreateObject<UniformRandomVariable>();
    zDist->SetAttribute("Min", DoubleValue(0.0));
    zDist->SetAttribute("Max", DoubleValue(30.0));

    NS_LOG_INFO("  STA spawn mode: uniform distribution across 400m x 400m playfield");

    Ptr<UniformRandomVariable> dwellRng = CreateObject<UniformRandomVariable>();
    const double waypointFieldSize = 400.0;

    for (uint32_t i = 0; i < numStaNodes; ++i)
    {
        double zValue = staHeight > 0.0 ? staHeight : zDist->GetValue();
        uint32_t nodeId = config.staNodes.Get(i)->GetId();

        if (mobilityModel == "waypoint")
        {
            double speed = (staSpeedMin + staSpeedMax) / 2.0;
            std::vector<Waypoint> route = BuildRobotWaypointRoute(
                robotType, speed, simTime, waypointFieldSize, zValue, i, numStaNodes, dwellRng);

            MobilityHelper wpMobilityHelper;
            wpMobilityHelper.SetMobilityModel("ns3::WaypointMobilityModel");
            wpMobilityHelper.Install(config.staNodes.Get(i));

            Ptr<WaypointMobilityModel> wpModel =
                config.staNodes.Get(i)->GetObject<WaypointMobilityModel>();
            for (const Waypoint& wp : route)
            {
                wpModel->AddWaypoint(wp);
            }

            Vector staPos = route.empty() ? Vector(0.0, 0.0, zValue) : route.front().position;
            std::cout << "Waypoint STA " << (i + 1) << " (" << robotType << ") route start ("
                      << staPos.x << ", " << staPos.y << ", " << staPos.z
                      << ") [nodeId " << nodeId << "], " << route.size() << " waypoints"
                      << std::endl;
            NS_LOG_INFO("  STA " << (i + 1) << " (nodeId " << nodeId << ") waypoint route ("
                        << robotType << "), " << route.size() << " waypoints, start ("
                        << staPos.x << ", " << staPos.y << ", " << staPos.z << ")");
            continue;
        }

        Vector staPos(xDist->GetValue(), yDist->GetValue(), zValue);

        Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
        posAlloc->Add(staPos);

        MobilityHelper staMobilityHelper;
        staMobilityHelper.SetPositionAllocator(posAlloc);
        std::ostringstream meanVelocity;
        meanVelocity << "ns3::UniformRandomVariable[Min=" << staSpeedMin
                     << "|Max=" << staSpeedMax << "]";
        staMobilityHelper.SetMobilityModel("ns3::GaussMarkovMobilityModel",
            "Bounds", BoxValue(Box(0.0, 400.0,
                                    0.0, 400.0,
                                    0.0, 30.0)),
            "TimeStep", TimeValue(Seconds(1.0)),
            "Alpha", DoubleValue(0.85),
            "MeanVelocity", StringValue(meanVelocity.str()),
            "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
            "MeanPitch", StringValue("ns3::UniformRandomVariable[Min=-0.05|Max=0.05]"),
            "NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
            "NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.1|Bound=0.2]"),
            "NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.01|Bound=0.02]"));
        staMobilityHelper.Install(config.staNodes.Get(i));

        std::cout << "Uniform STA spawn " << (i + 1) << " at ("
                  << staPos.x << ", " << staPos.y << ", " << staPos.z
                  << ") [nodeId " << nodeId << "]" << std::endl;
        NS_LOG_INFO("  STA " << (i + 1) << " (nodeId " << nodeId
                    << ") initial position ("
                    << staPos.x << ", " << staPos.y << ", " << staPos.z << ")");
    }

    BuildingsHelper::Install(config.staNodes);

    return config;
}

// Give each WiFi STA node an LTE interface using a single macro eNB.
// This sets up LTE/EPC and assigns LTE IPv4 addresses to each UE. It also
// connects PGW to the existing ISP router so UEs can reach the same service IP
// over either WiFi or LTE; per-UE route steering is handled by the hybrid controller.
LteHybridConfig SetupLteHybridOverlay(const NodeContainer& ueNodes,
                                      Ptr<Node> ispRouter,
                                      Ipv4Address serviceIp,
                                      double fieldSize)
{
    NS_LOG_FUNCTION("Setting up LTE overlay for " << ueNodes.GetN() << " UEs");

    LteHybridConfig cfg;
    if (ueNodes.GetN() == 0)
    {
        return cfg;
    }

    // One macro eNB at the center of the 400 x 400 m playfield
    cfg.enbNodes.Create(1);

    MobilityHelper enbMobility;
    Ptr<ListPositionAllocator> enbPosAlloc = CreateObject<ListPositionAllocator>();
    enbPosAlloc->Add(Vector(fieldSize / 2.0, fieldSize / 2.0, 25.0));
    enbMobility.SetPositionAllocator(enbPosAlloc);
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.Install(cfg.enbNodes);

    BuildingsHelper::Install(cfg.enbNodes);

    // LTE PHY configuration aligned with 4G_simulation_file.cc
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(43.0)); // dBm
    Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(15.0));  // dBm

    cfg.lteHelper = CreateObject<LteHelper>();
    cfg.epcHelper = CreateObject<PointToPointEpcHelper>();
    cfg.lteHelper->SetEpcHelper(cfg.epcHelper);

    cfg.lteHelper->SetPathlossModelType(HybridBuildingsPropagationLossModel::GetTypeId());
    cfg.lteHelper->SetPathlossModelAttribute("Frequency", DoubleValue(2.0e9));
    cfg.lteHelper->SetUeAntennaModelType("ns3::IsotropicAntennaModel");
    cfg.lteHelper->SetUeAntennaModelAttribute("Gain", DoubleValue(1.0));

    cfg.enbDevices = cfg.lteHelper->InstallEnbDevice(cfg.enbNodes);
    cfg.ueDevices = cfg.lteHelper->InstallUeDevice(ueNodes);

    // Assign LTE IPv4 addresses to the LTE interfaces on each UE.
    cfg.ueIfaces = cfg.epcHelper->AssignUeIpv4Address(cfg.ueDevices);

    // Attach all UEs to the single eNB (no handover in this topology)
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        cfg.lteHelper->Attach(cfg.ueDevices.Get(i), cfg.enbDevices.Get(0));
    }

    cfg.ueGateway = cfg.epcHelper->GetUeDefaultGatewayAddress();
    cfg.serviceIp = serviceIp;

    // Connect PGW to the existing ISP router, so LTE UEs can reach the same service IP.
    Ptr<Node> pgw = cfg.epcHelper->GetPgwNode();

    // Add simple mobility to PGW to avoid warnings
    NodeContainer epcCore;
    epcCore.Add(pgw);
    MobilityHelper epcMobility;
    Ptr<ListPositionAllocator> epcPos = CreateObject<ListPositionAllocator>();
    epcPos->Add(Vector(fieldSize * 0.5, fieldSize + 100.0, 0.0));
    epcMobility.SetPositionAllocator(epcPos);
    epcMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    epcMobility.Install(epcCore);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Gbps"));
    p2p.SetChannelAttribute("Delay", StringValue("1ms"));
    NetDeviceContainer pgwIspDevices = p2p.Install(pgw, ispRouter);

    // Address the PGW<->ISP link
    Ipv4AddressHelper p2pAddr;
    p2pAddr.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer pgwIspIf = p2pAddr.Assign(pgwIspDevices);
    Ipv4Address pgwP2pIp = pgwIspIf.GetAddress(0);
    Ipv4Address ispP2pIp = pgwIspIf.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    // ISP router route back to LTE UE subnet (EPC defaults to 7.0.0.0/8)
    {
        Ptr<Ipv4StaticRouting> ispRouting =
            ipv4RoutingHelper.GetStaticRouting(ispRouter->GetObject<Ipv4>());
        uint32_t ispIf = ispRouter->GetObject<Ipv4>()->GetInterfaceForDevice(pgwIspDevices.Get(1));
        ispRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                      Ipv4Mask("255.0.0.0"),
                                      pgwP2pIp,
                                      ispIf);
    }

    // PGW route to the service network (8.8.8.0/24) via ISP router
    {
        Ptr<Ipv4StaticRouting> pgwRouting =
            ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
        uint32_t pgwIf = pgw->GetObject<Ipv4>()->GetInterfaceForDevice(pgwIspDevices.Get(0));
        pgwRouting->AddNetworkRouteTo(Ipv4Address("8.8.8.0"),
                                      Ipv4Mask("255.255.255.0"),
                                      ispP2pIp,
                                      pgwIf);
    }

    return cfg;
}

// Give each WiFi STA node an NR (5G) interface using a single macro gNB.
// This sets up NR/EPC and assigns NR IPv4 addresses to each UE, then connects
// PGW to ISP router so UEs can reach the same service IP over WiFi or NR.
NrHybridConfig SetupNrHybridOverlay(const NodeContainer& ueNodes,
                                    Ptr<Node> ispRouter,
                                    Ipv4Address serviceIp,
                                    double fieldSize)
{
    NS_LOG_FUNCTION("Setting up NR overlay for " << ueNodes.GetN() << " UEs");

    NrHybridConfig cfg;
    if (ueNodes.GetN() == 0)
    {
        return cfg;
    }

    cfg.gnbNodes.Create(1);
    MobilityHelper gnbMobility;
    Ptr<ListPositionAllocator> gnbPosAlloc = CreateObject<ListPositionAllocator>();
    gnbPosAlloc->Add(Vector(fieldSize / 2.0, fieldSize / 2.0, 25.0));
    gnbMobility.SetPositionAllocator(gnbPosAlloc);
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install(cfg.gnbNodes);
    BuildingsHelper::Install(cfg.gnbNodes);

    cfg.nrHelper = CreateObject<NrHelper>();
    cfg.epcHelper = CreateObject<NrPointToPointEpcHelper>();
    cfg.nrHelper->SetEpcHelper(cfg.epcHelper);

    // Align UE TX power with hotspot client profile.
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(15.0));
    cfg.nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    cfg.nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    Ptr<IsotropicAntennaModel> ueAntenna = CreateObject<IsotropicAntennaModel>();
    ueAntenna->SetAttribute("Gain", DoubleValue(1.0));
    cfg.nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(ueAntenna));

    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    cfg.nrHelper->SetBeamformingHelper(beamformingHelper);
    beamformingHelper->SetAttribute("BeamformingMethod",
                                    TypeIdValue(DirectPathBeamforming::GetTypeId()));

    const double centralFrequency = 3.5e9;
    const double bandwidth = 100e6;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    bandConf.m_numBwp = 1;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("RMa", "Default", "ThreeGpp");
    channelHelper->ConfigurePropagationFactory(HybridBuildingsPropagationLossModel::GetTypeId());
    channelHelper->SetPathlossAttribute("Frequency", DoubleValue(centralFrequency));
    channelHelper->AssignChannelsToBands({band});

    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});
    cfg.gnbDevices = cfg.nrHelper->InstallGnbDevice(cfg.gnbNodes, allBwps);
    cfg.ueDevices = cfg.nrHelper->InstallUeDevice(ueNodes, allBwps);
    cfg.ueIfaces = cfg.epcHelper->AssignUeIpv4Address(NetDeviceContainer(cfg.ueDevices));
    cfg.nrHelper->AttachToClosestGnb(cfg.ueDevices, cfg.gnbDevices);

    cfg.ueGateway = cfg.epcHelper->GetUeDefaultGatewayAddress();
    cfg.serviceIp = serviceIp;

    Ptr<Node> pgw = cfg.epcHelper->GetPgwNode();
    NodeContainer epcCore;
    epcCore.Add(pgw);
    MobilityHelper epcMobility;
    Ptr<ListPositionAllocator> epcPos = CreateObject<ListPositionAllocator>();
    epcPos->Add(Vector(fieldSize * 0.5, fieldSize + 100.0, 0.0));
    epcMobility.SetPositionAllocator(epcPos);
    epcMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    epcMobility.Install(epcCore);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Gbps"));
    p2p.SetChannelAttribute("Delay", StringValue("1ms"));
    NetDeviceContainer pgwIspDevices = p2p.Install(pgw, ispRouter);

    Ipv4AddressHelper p2pAddr;
    p2pAddr.SetBase("2.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer pgwIspIf = p2pAddr.Assign(pgwIspDevices);
    Ipv4Address pgwP2pIp = pgwIspIf.GetAddress(0);
    Ipv4Address ispP2pIp = pgwIspIf.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    {
        Ptr<Ipv4StaticRouting> ispRouting =
            ipv4RoutingHelper.GetStaticRouting(ispRouter->GetObject<Ipv4>());
        uint32_t ispIf = ispRouter->GetObject<Ipv4>()->GetInterfaceForDevice(pgwIspDevices.Get(1));
        ispRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                      Ipv4Mask("255.0.0.0"),
                                      pgwP2pIp,
                                      ispIf);
    }

    {
        Ptr<Ipv4StaticRouting> pgwRouting =
            ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
        uint32_t pgwIf = pgw->GetObject<Ipv4>()->GetInterfaceForDevice(pgwIspDevices.Get(0));
        pgwRouting->AddNetworkRouteTo(Ipv4Address("8.8.8.0"),
                                      Ipv4Mask("255.255.255.0"),
                                      ispP2pIp,
                                      pgwIf);
    }

    return cfg;
}

void ConfigureIPForwarding(NodeContainer meshNodes, NodeContainer internetNodes)
{
    NS_LOG_FUNCTION("Configuring IP forwarding");
    
    // Enable IP forwarding on all mesh nodes (each may host an AP)
    for (uint32_t i = 0; i < meshNodes.GetN(); ++i)
    {
        Ptr<Ipv4> ipv4Mesh = meshNodes.Get(i)->GetObject<Ipv4>();
        ipv4Mesh->SetAttribute("IpForward", BooleanValue(true));
    }

    // Enable IP forwarding on ISP router
    Ptr<Ipv4> ipv4Router = internetNodes.Get(0)->GetObject<Ipv4>();
    ipv4Router->SetAttribute("IpForward", BooleanValue(true));
}

void ConfigureStaticRouting(const MeshNetworkConfig& meshConfig,
                            const InternetConfig& internetConfig,
                            const HotspotConfig& hotspotConfig,
                            uint32_t nNodes,
                            bool enableHotspot)
{
    NS_LOG_FUNCTION("Configuring static routing");
    
    Ipv4StaticRoutingHelper staticRouting;

    // --- Configure all mesh nodes ---
    for (uint32_t i = 0; i < nNodes; i++)
    {
        Ptr<Ipv4> ipv4 = meshConfig.meshNodes.Get(i)->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> routing = staticRouting.GetStaticRouting(ipv4);
        
        if (i == 0)
        {
            // Gateway Node (Node 0) - route internet traffic via Ethernet
            Ipv4Address nextHopIp = internetConfig.backhaulInterfaces.GetAddress(1);
            Ptr<NetDevice> gatewayCsmaDevice = internetConfig.backhaulDevices.Get(0);
            uint32_t gatewayCsmaInterface = ipv4->GetInterfaceForDevice(gatewayCsmaDevice);
            
            NS_LOG_INFO("Gateway (Node 0) default route to " << nextHopIp 
                       << " via interface " << gatewayCsmaInterface);
            routing->SetDefaultRoute(nextHopIp, gatewayCsmaInterface);
            
            if (enableHotspot)
            {
                NS_LOG_INFO("Gateway host routes to STAs will be managed dynamically on association events");
            }
        }
        else
        {
            // Regular Mesh Nodes - route all traffic to Gateway (Node 0)
            Ipv4Address gatewayMeshIp = meshConfig.meshInterfaces.GetAddress(0);
            Ptr<NetDevice> nodeMeshDevice = meshConfig.meshDevices.Get(i);
            uint32_t nodeMeshInterface = ipv4->GetInterfaceForDevice(nodeMeshDevice);
            routing->SetDefaultRoute(gatewayMeshIp, nodeMeshInterface);
        }
    }
    
    // --- Configure the ISP Router ---
    Ptr<Ipv4> ipv4IspRouter = internetConfig.internetNodes.Get(0)->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> ispRouting = staticRouting.GetStaticRouting(ipv4IspRouter);
    
    Ipv4Address gatewayBackhaulIp = internetConfig.backhaulInterfaces.GetAddress(0);
    Ptr<NetDevice> ispCsmaDevice = internetConfig.backhaulDevices.Get(1);
    uint32_t ispBackhaulInterface = ipv4IspRouter->GetInterfaceForDevice(ispCsmaDevice);
    
    ispRouting->AddNetworkRouteTo(Ipv4Address("10.1.1.0"),
                                  Ipv4Mask("255.255.255.0"),
                                  gatewayBackhaulIp,
                                  ispBackhaulInterface);
    
    if (enableHotspot)
    {
        ispRouting->AddNetworkRouteTo(Ipv4Address("192.168.1.0"),
                                      Ipv4Mask("255.255.255.0"),
                                      gatewayBackhaulIp,
                                      ispBackhaulInterface);
        NS_LOG_INFO("ISP Router route to hotspot network (192.168.1.0/24) via gateway backhaul IP "
                    << gatewayBackhaulIp);
    }
    
    // --- Configure the Internet Server ---
    Ptr<Ipv4> ipv4Server = internetConfig.internetNodes.Get(1)->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> serverRouting = staticRouting.GetStaticRouting(ipv4Server);
    
    Ipv4Address ispRouterIp = internetConfig.internetInterfaces.GetAddress(0);
    Ptr<NetDevice> serverCsmaDevice = internetConfig.backboneDevices.Get(1);
    uint32_t serverInterface = ipv4Server->GetInterfaceForDevice(serverCsmaDevice);
    
    serverRouting->SetDefaultRoute(ispRouterIp, serverInterface);
}

// -----------------------------------------------------------------------------
// July enhancement: per-flow QoS-separated traffic model.
//
// Instead of the generic HTTP/HTTPS/Video/VoIP mix, robot communication is
// modeled as three flows with distinct QoS requirements and DSCP markings so
// that FlowMonitor (and flow_metrics.py) can measure each flow independently:
//
//   Flow      Transport  Pattern                         DSCP (ToS)
//   --------  ---------  ------------------------------   ----------------
//   Control   UDP        small (~1 KB) every 50 ms        EF   (46) -> 0xB8
//   Sensor    TCP        periodic 100 KB every 200 ms     AF31 (26) -> 0x68
//   Video     TCP        continuous 5 Mbps stream         AF41 (34) -> 0x88
//
// DSCP is applied via the OnOffApplication "Tos" attribute, which the app
// copies into the socket's IP_TOS field (SetIpTos) at start.
// -----------------------------------------------------------------------------
namespace
{
// ToS byte = DSCP << 2 (ECN bits left zero).
constexpr uint8_t QOS_TOS_CONTROL = 46 << 2; // EF
constexpr uint8_t QOS_TOS_SENSOR = 26 << 2;  // AF31
constexpr uint8_t QOS_TOS_VIDEO = 34 << 2;   // AF41
} // namespace

void SetupApplications(const MeshNetworkConfig& meshConfig,
                      const InternetConfig& internetConfig,
                      const HotspotConfig& hotspotConfig,
                      double simTime,
                      bool enableHotspot,
                      uint32_t packetSize,
                      uint32_t uploadBytes,
                      uint32_t downloadBytes,
                      uint32_t voipBytes,
                      double flowScale)
{
    NS_LOG_FUNCTION("Setting up QoS-separated applications");

    (void)packetSize;
    (void)uploadBytes;
    (void)downloadBytes;
    (void)voipBytes;

    // Per-flow port bases so FlowMonitor separates the three robot flows.
    const uint16_t controlPortBase = 52000; // Control commands (UDP)
    const uint16_t sensorPortBase = 53000;  // Sensor / telemetry (TCP)
    const uint16_t videoPortBase = 54000;   // Video stream (TCP)

    // Flow parameters (per STA). Control stays fixed (robot-safety traffic).
    // Sensor/Video rates scale with flowScale so the matrix payload factor
    // (10kb/50kb/1mb/2mb → 0.1/0.25/1.0/2.0) changes offered load.
    const double scale = std::max(0.01, flowScale);
    const uint32_t controlPacketBytes = 1024;      // ~1 KB control command
    const std::string controlDataRate = "164kbps"; // 1024 B / 50 ms
    const uint32_t sensorChunkBytes = 100 * 1024;  // 100 KB telemetry burst
    const uint64_t sensorBps = static_cast<uint64_t>(std::llround(4.0e6 * scale));
    const std::string sensorDataRate = std::to_string(sensorBps) + "bps";
    const uint32_t sensorPacketBytes = 1400;
    const uint64_t videoBps = static_cast<uint64_t>(std::llround(5.0e6 * scale));
    const std::string videoDataRate = std::to_string(videoBps) + "bps";
    const uint32_t videoPacketBytes = 1400;

    Ipv4Address serverIp = internetConfig.internetInterfaces.GetAddress(1);
    Ptr<Node> serverNode = internetConfig.internetNodes.Get(1);

    if (!enableHotspot)
    {
        return;
    }

    const uint32_t staCount = hotspotConfig.staNodes.GetN();

    for (uint32_t i = 0; i < staCount; ++i)
    {
        double baseStart = 10.0 + static_cast<double>(i) * 0.4;
        uint16_t controlPort = controlPortBase + i;
        uint16_t sensorPort = sensorPortBase + i;
        uint16_t videoPort = videoPortBase + i;

        // --- Control flow (UDP, EF) ------------------------------------------
        UdpServerHelper controlServer(controlPort);
        ApplicationContainer controlServerApp = controlServer.Install(serverNode);
        controlServerApp.Start(Seconds(0.5));
        controlServerApp.Stop(Seconds(simTime));
        // Use the continuous Control UDP flow's server RX as the switch-recovery
        // liveness signal (accurate, fast-recovering; see HandleControlLivenessRx).
        if (controlServerApp.GetN() > 0)
        {
            controlServerApp.Get(0)->TraceConnectWithoutContext(
                "Rx", MakeBoundCallback(&HandleControlLivenessRx, i));
        }

        OnOffHelper controlClient("ns3::UdpSocketFactory",
                                  InetSocketAddress(serverIp, controlPort));
        controlClient.SetAttribute("Tos", UintegerValue(QOS_TOS_CONTROL));
        controlClient.SetAttribute("DataRate", DataRateValue(DataRate(controlDataRate)));
        controlClient.SetAttribute("PacketSize", UintegerValue(controlPacketBytes));
        controlClient.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        controlClient.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        ApplicationContainer controlApp = controlClient.Install(hotspotConfig.staNodes.Get(i));
        controlApp.Start(Seconds(baseStart + 0.1));
        controlApp.Stop(Seconds(simTime));
        // Control is UDP fire-and-forget; not tracked in the PDR switch window
        // (same rationale as the legacy VoIP flow) to avoid spurious switches.

        // --- Sensor / telemetry flow (TCP, AF31) -----------------------------
        // This TCP flow drives the PDR-based switching window (reliable, steerable
        // by the same WiFi/cellular host route).
        OnOffHelper sensorClient("ns3::TcpSocketFactory",
                                 InetSocketAddress(serverIp, sensorPort));
        sensorClient.SetAttribute("Tos", UintegerValue(QOS_TOS_SENSOR));
        sensorClient.SetAttribute("DataRate", DataRateValue(DataRate(sensorDataRate)));
        sensorClient.SetAttribute("PacketSize", UintegerValue(sensorPacketBytes));
        sensorClient.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        sensorClient.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        sensorClient.SetAttribute("MaxBytes", UintegerValue(0)); // continuous periodic upload
        ApplicationContainer sensorApp = sensorClient.Install(hotspotConfig.staNodes.Get(i));
        sensorApp.Start(Seconds(baseStart + 0.2));
        sensorApp.Stop(Seconds(simTime));
        if (sensorApp.GetN() > 0)
        {
            sensorApp.Get(0)->TraceConnectWithoutContext("Tx",
                                                         MakeBoundCallback(&HandleServiceTx, i));
        }
        (void)sensorChunkBytes;

        PacketSinkHelper sensorSink("ns3::TcpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), sensorPort));
        ApplicationContainer sensorSinkApp = sensorSink.Install(serverNode);
        sensorSinkApp.Start(Seconds(9.5));
        sensorSinkApp.Stop(Seconds(simTime));
        Ptr<PacketSink> sensorPacketSink = DynamicCast<PacketSink>(sensorSinkApp.Get(0));
        if (sensorPacketSink)
        {
            sensorPacketSink->TraceConnectWithoutContext("Rx",
                                                         MakeBoundCallback(&HandleServiceSinkRx, i));
        }

        // --- Video flow (TCP, AF41) ------------------------------------------
        OnOffHelper videoClient("ns3::TcpSocketFactory",
                                InetSocketAddress(serverIp, videoPort));
        videoClient.SetAttribute("Tos", UintegerValue(QOS_TOS_VIDEO));
        videoClient.SetAttribute("DataRate", DataRateValue(DataRate(videoDataRate)));
        videoClient.SetAttribute("PacketSize", UintegerValue(videoPacketBytes));
        videoClient.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        videoClient.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        videoClient.SetAttribute("MaxBytes", UintegerValue(0));
        ApplicationContainer videoApp = videoClient.Install(hotspotConfig.staNodes.Get(i));
        videoApp.Start(Seconds(baseStart + 0.3));
        videoApp.Stop(Seconds(simTime));

        PacketSinkHelper videoSink("ns3::TcpSocketFactory",
                                   InetSocketAddress(Ipv4Address::GetAny(), videoPort));
        ApplicationContainer videoSinkApp = videoSink.Install(serverNode);
        videoSinkApp.Start(Seconds(9.5));
        videoSinkApp.Stop(Seconds(simTime));
    }
}

void SaveFlowMonitorResults(Ptr<FlowMonitor> monitor,
                            FlowMonitorHelper& flowmon,
                            const std::string& flowmonXmlPath)
{
    NS_LOG_FUNCTION("Saving FlowMonitor results");
    
    monitor->CheckForLostPackets();
    
    // Save to XML file
    monitor->SerializeToXmlFile(flowmonXmlPath, true, true);
    
    // Print statistics to console
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(
        flowmon.GetClassifier());
    
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    
    double totalTxBytes = 0;
    double totalRxBytes = 0;
    double totalTxPackets = 0;
    double totalRxPackets = 0;
    double totalLostPackets = 0;
    double totalDelay = 0;
    uint32_t flowCount = 0;
    
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
         i != stats.end(); ++i)
    {
        flowCount++;
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        
        NS_LOG_UNCOND("Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort 
                      << " -> " << t.destinationAddress << ":" << t.destinationPort << ")");
        NS_LOG_UNCOND("  Protocol: " << (uint32_t)t.protocol);
        NS_LOG_UNCOND("  Tx Packets: " << i->second.txPackets);
        NS_LOG_UNCOND("  Tx Bytes:   " << (i->second.txBytes / 1048576.0) << " MB");
        NS_LOG_UNCOND("  Rx Packets: " << i->second.rxPackets);
        NS_LOG_UNCOND("  Rx Bytes:   " << (i->second.rxBytes / 1048576.0) << " MB");
        NS_LOG_UNCOND("  Lost Packets: " << i->second.lostPackets 
                      << " (" << (i->second.txPackets > 0 ? 
                      (100.0 * i->second.lostPackets / i->second.txPackets) : 0) << "%)");
        
        if (i->second.rxPackets > 0)
        {
            double avgDelay = i->second.delaySum.GetSeconds() / i->second.rxPackets;
            double avgJitter = (i->second.rxPackets > 1) ? 
                i->second.jitterSum.GetSeconds() / (i->second.rxPackets - 1) : 0.0;
            double timeDiff = i->second.timeLastRxPacket.GetSeconds() - 
                              i->second.timeFirstTxPacket.GetSeconds();
            double throughput = (timeDiff > 0.0) ? 
                (i->second.rxBytes * 8.0 / timeDiff / 1048576.0) : 0.0; // Mbps
            
            NS_LOG_UNCOND("  Throughput: " << throughput << " Mbps");
            NS_LOG_UNCOND("  Avg Delay:  " << (avgDelay * 1000.0) << " ms");
            NS_LOG_UNCOND("  Avg Jitter: " << (avgJitter * 1000.0) << " ms");
            
            totalDelay += avgDelay;
        }
        else
        {
             NS_LOG_UNCOND("  Throughput: 0 Mbps (no packets received)");
             NS_LOG_UNCOND("  Avg Delay:  N/A");
             NS_LOG_UNCOND("  Avg Jitter: N/A");
        }
        
        totalTxBytes += i->second.txBytes;
        totalRxBytes += i->second.rxBytes;
        totalTxPackets += i->second.txPackets;
        totalRxPackets += i->second.rxPackets;
        totalLostPackets += i->second.lostPackets;
    }
    
    NS_LOG_UNCOND("  Total Flows: " << flowCount);
    NS_LOG_UNCOND("  Total Tx Packets: " << totalTxPackets);
    NS_LOG_UNCOND("  Total Rx Packets: " << totalRxPackets);
    NS_LOG_UNCOND("  Total Lost Packets: " << totalLostPackets 
                  << " (" << (totalTxPackets > 0 ? 
                  (100.0 * totalLostPackets / totalTxPackets) : 0) << "%)");
    NS_LOG_UNCOND("  Total Tx Bytes: " << (totalTxBytes / 1048576.0) << " MB");
    NS_LOG_UNCOND("  Total Rx Bytes: " << (totalRxBytes / 1048576.0) << " MB");
    if (flowCount > 0 && totalRxPackets > 0)
    {
        NS_LOG_UNCOND("  Average Delay: " << ((totalDelay / flowCount) * 1000.0) << " ms");
    }
    
    NS_LOG_INFO("FlowMonitor results saved to XML and printed to console");
}

// ---------------------------------------------------------------------------//
// WiFi RSSI monitoring for hybrid controller
// ---------------------------------------------------------------------------//

static void
WifiSnifferRxCallback(std::string context,
                      Ptr<const Packet> packet,
                      uint16_t channelFreqMhz,
                      WifiTxVector txVector,
                      MpduInfo aMpdu,
                      SignalNoiseDbm signalNoise,
                      uint16_t /*channelNumber*/)
{
    if (!g_hybridContext.enabled || g_hybridContext.nodeIdToStaIndex == nullptr)
    {
        return;
    }

    uint32_t nodeId = 0;
    if (!ExtractNodeIdFromContext(context, nodeId))
    {
        return;
    }

    auto it = g_hybridContext.nodeIdToStaIndex->find(nodeId);
    if (it == g_hybridContext.nodeIdToStaIndex->end())
    {
        // Not a STA node we track
        return;
    }
    uint32_t staIndex = it->second;
    if (staIndex >= g_hybridContext.ueStates.size())
    {
        return;
    }

    // Track RSSI from any known AP BSSID (not just the current association).
    // This enables fallback AP selection when currentStaApIndex is temporarily unknown.
    if (g_hybridContext.hotspotConfig == nullptr)
    {
        return;
    }
    const HotspotConfig& hotspot = *g_hybridContext.hotspotConfig;

    // Extract transmitter MAC address from the 802.11 header.
    WifiMacHeader hdr;
    Ptr<Packet> pktCopy = packet->Copy();
    if (!pktCopy->PeekHeader(hdr))
    {
        return;
    }
    Mac48Address txAddr = hdr.GetAddr2(); // transmitter address

    auto apIt = hotspot.apBssidToMeshIndex.find(txAddr);
    if (apIt == hotspot.apBssidToMeshIndex.end())
    {
        // Ignore frames not sent by one of our AP BSSIDs
        return;
    }
    uint32_t apIndex = apIt->second;

    double rssiDbm = signalNoise.signal;
    HybridUeState& st = g_hybridContext.ueStates[staIndex];

    if (st.apRssiAvgDbm.size() != hotspot.apInterfaces.GetN())
    {
        st.apRssiAvgDbm.assign(hotspot.apInterfaces.GetN(), -1000.0);
        st.apRssiLastUpdate.assign(hotspot.apInterfaces.GetN(), Seconds(0.0));
    }

    const double alphaAp = 0.8;
    if (st.apRssiLastUpdate[apIndex] == Seconds(0.0))
    {
        st.apRssiAvgDbm[apIndex] = rssiDbm;
    }
    else
    {
        st.apRssiAvgDbm[apIndex] = alphaAp * st.apRssiAvgDbm[apIndex] + (1.0 - alphaAp) * rssiDbm;
    }
    st.apRssiLastUpdate[apIndex] = Simulator::Now();

    const double alpha = 0.6; // balanced: tracks movement in ~5 samples at 0.5s interval
    // Any WiFi frame heard means we're in range — update the liveness timestamp.
    st.lastRssiSampleTime = Simulator::Now();
    if (staIndex < hotspot.currentStaApIndex.size())
    {
        uint32_t assocAp = hotspot.currentStaApIndex[staIndex];
        if (assocAp != std::numeric_limits<uint32_t>::max() && assocAp == apIndex)
        {
            // Associated: track RSSI from the serving AP only.
            if (!st.rssiInitialized)
            {
                st.rssiAvgDbm = rssiDbm;
                st.rssiInitialized = true;
            }
            else
            {
                st.rssiAvgDbm = alpha * st.rssiAvgDbm + (1.0 - alpha) * rssiDbm;
            }
        }
        else if (assocAp == std::numeric_limits<uint32_t>::max())
        {
            // Not associated (e.g. while on cellular): keep rssiAvgDbm from going
            // stale by updating it upward from any visible AP beacon. This prevents
            // a frozen bad-RSSI value from blocking cellular->WiFi return.
            if (!st.rssiInitialized)
            {
                st.rssiAvgDbm = rssiDbm;
                st.rssiInitialized = true;
            }
            else if (rssiDbm > st.rssiAvgDbm)
            {
                st.rssiAvgDbm = alpha * st.rssiAvgDbm + (1.0 - alpha) * rssiDbm;
            }
        }
    }

    if (g_rssiLogOpen)
    {
        // Keep the original first 6 columns intact. Append WiFi-specific fields.
        g_rssiLog << Simulator::Now().GetSeconds() << ","
                  << staIndex << ","
                  << nodeId << ","
                  << channelFreqMhz << ","
                  << rssiDbm << ","
                  << (st.rssiInitialized ? st.rssiAvgDbm : rssiDbm) << ","
                  << "wifi" << ","
                  << "nan" << ","
                  << "nan" << ","
                  << "nan" << ","
                  << "nan" << ","
                  << "nan" << ","
                  << (st.currentPath == USE_WIFI ? "wifi" : "cellular") << ","
                  << GetStaPositionCsv(staIndex)
                  << "\n";
    }
}

// ---------------------------------------------------------------------------//
// Cellular link quality monitoring (LTE/NR) for hybrid controller
// ---------------------------------------------------------------------------//

static void
LteUeRsrpSinrCallback(std::string context,
                      uint16_t cellId,
                      uint16_t rnti,
                      double rsrpW,
                      double sinrLinear,
                      uint8_t componentCarrierId)
{
    if (!g_hybridContext.enabled || g_hybridContext.nodeIdToStaIndex == nullptr || !g_rssiLogOpen)
    {
        return;
    }

    uint32_t nodeId = 0;
    if (!ExtractNodeIdFromContext(context, nodeId))
    {
        return;
    }

    auto it = g_hybridContext.nodeIdToStaIndex->find(nodeId);
    if (it == g_hybridContext.nodeIdToStaIndex->end())
    {
        return;
    }
    uint32_t staIndex = it->second;
    if (staIndex >= g_hybridContext.ueStates.size())
    {
        return;
    }

    // LTE trace reports RSRP in linear Watts and SINR in linear scale.
    const double rsrpDbm = 10.0 * std::log10(rsrpW) + 30.0;
    const double sinrDb = 10.0 * std::log10(sinrLinear);
    const HybridUeState& st = g_hybridContext.ueStates[staIndex];

    // Append to the same CSV as WiFi RSSI, keeping first 6 columns intact.
    g_rssiLog << Simulator::Now().GetSeconds() << ","
              << staIndex << ","
              << nodeId << ","
              << 0 << ","
              << "nan" << ","
              << "nan" << ","
              << "lte" << ","
              << cellId << ","
              << rnti << ","
              << static_cast<uint32_t>(componentCarrierId) << ","
              << rsrpDbm << ","
              << sinrDb << ","
              << (st.currentPath == USE_WIFI ? "wifi" : "cellular") << ","
              << GetStaPositionCsv(staIndex)
              << "\n";
}

static void
NrUeRsrpCallback(std::string context,
                 uint16_t cellId,
                 uint16_t ueId,
                 uint16_t rnti,
                 double rsrpDbm,
                 uint8_t bwpId)
{
    if (!g_hybridContext.enabled || g_hybridContext.nodeIdToStaIndex == nullptr || !g_rssiLogOpen)
    {
        return;
    }

    uint32_t nodeId = 0;
    if (!ExtractNodeIdFromContext(context, nodeId))
    {
        return;
    }

    auto it = g_hybridContext.nodeIdToStaIndex->find(nodeId);
    if (it == g_hybridContext.nodeIdToStaIndex->end())
    {
        return;
    }
    uint32_t staIndex = it->second;
    if (staIndex >= g_hybridContext.ueStates.size())
    {
        return;
    }

    const HybridUeState& st = g_hybridContext.ueStates[staIndex];

    // NR trace reports RSRP already in dBm (see contrib/nr/model/nr-ue-phy.cc).
    g_rssiLog << Simulator::Now().GetSeconds() << ","
              << staIndex << ","
              << nodeId << ","
              << 0 << ","
              << "nan" << ","
              << "nan" << ","
              << "nr" << ","
              << cellId << ","
              << rnti << ","
              << static_cast<uint32_t>(bwpId) << ","
              << rsrpDbm << ","
              << "nan" << ","
              << (st.currentPath == USE_WIFI ? "wifi" : "cellular") << ","
              << GetStaPositionCsv(staIndex)
              << "\n";

    (void)ueId;
}

static void
NrUeDlDataSinrCallback(std::string context,
                       uint16_t cellId,
                       uint16_t rnti,
                       double sinrLinear,
                       uint16_t bwpId)
{
    if (!g_hybridContext.enabled || g_hybridContext.nodeIdToStaIndex == nullptr || !g_rssiLogOpen)
    {
        return;
    }

    uint32_t nodeId = 0;
    if (!ExtractNodeIdFromContext(context, nodeId))
    {
        return;
    }

    auto it = g_hybridContext.nodeIdToStaIndex->find(nodeId);
    if (it == g_hybridContext.nodeIdToStaIndex->end())
    {
        return;
    }
    uint32_t staIndex = it->second;
    if (staIndex >= g_hybridContext.ueStates.size())
    {
        return;
    }

    const HybridUeState& st = g_hybridContext.ueStates[staIndex];
    const double sinrDb = 10.0 * std::log10(sinrLinear);

    g_rssiLog << Simulator::Now().GetSeconds() << ","
              << staIndex << ","
              << nodeId << ","
              << 0 << ","
              << "nan" << ","
              << "nan" << ","
              << "nr" << ","
              << cellId << ","
              << rnti << ","
              << bwpId << ","
              << "nan" << ","
              << sinrDb << ","
              << (st.currentPath == USE_WIFI ? "wifi" : "cellular") << ","
              << GetStaPositionCsv(staIndex)
              << "\n";
}

void SaveConfigurationJSON(uint32_t nNodes, uint32_t gridWidth, uint32_t numStaNodes,
                           uint32_t packetSize, double nodeSpacing, uint32_t meshConfig,
                           const std::string& outputDir,
                           const std::string& mobilityModel,
                           const std::string& robotType)
{
    std::stringstream json;
    json << "{\n";
    json << "  \"network_topology\": {\n";
    json << "    \"num_nodes\": " << nNodes << ",\n";
    json << "    \"grid_width\": " << gridWidth << ",\n";
    json << "    \"node_spacing_meters\": " << nodeSpacing << "\n";
    json << "  },\n";
    json << "  \"traffic_configuration\": {\n";
    json << "    \"packet_size_bytes\": " << packetSize << ",\n";
    json << "    \"num_sta_nodes\": " << numStaNodes << "\n";
    json << "  },\n";
    json << "  \"mesh_configuration\": {\n";
    json << "    \"config_id\": " << meshConfig << "\n";
    json << "  },\n";
    json << "  \"mobility_configuration\": {\n";
    json << "    \"mobility_model\": \"" << mobilityModel << "\",\n";
    json << "    \"robot_type\": \"" << (mobilityModel == "waypoint" ? robotType : "n/a") << "\"\n";
    json << "  },\n";
    json << "  \"ip_configuration\": {\n";
    json << "    \"source_ip\": \"192.168.1.2\",\n";
    json << "    \"destination_ip\": \"8.8.8.2\"\n";
    json << "  },\n";
    json << "  \"port_information\": {\n";
    json << "    \"tcp_port\": 80,\n";
    json << "    \"udp_port\": 9,\n";
    json << "    \"note\": \"TCP and UDP (VoIP) traffic generated\"\n";
    json << "  },\n";
    json << "  \"output_files\": {\n";
    json << "    \"netanim_xml\": \"" << outputDir << "/wifi-hybrid-netanim_data.xml\",\n";
    json << "    \"mesh_tr\": \"" << outputDir << "/wifi-hybrid-mesh-trace_data.tr\",\n";
    json << "    \"sta_tr\": \"" << outputDir << "/wifi-hybrid-sta-trace_data.tr\",\n";
    json << "    \"flowmon_xml\": \"" << outputDir << "/wifi-hybrid-flowmon_data.xml\",\n";
    json << "    \"metrics_md\": \"" << outputDir << "/wifi-hybrid-metrics_data.md\"\n";
    json << "  }\n";
    json << "}\n";
    
    std::string filename = outputDir + "/config_test_2.json";
    std::ofstream configFile(filename);
    if (configFile.is_open())
    {
        configFile << json.str();
        configFile.close();
        NS_LOG_INFO("Configuration saved to " << filename);
    }
    else
    {
        NS_LOG_WARN("Unable to save configuration to " << filename);
    }
}

static void
SwitchUeToLte(uint32_t staIndex,
              const HotspotConfig& hotspotConfig,
              const LteHybridConfig& lteConfig,
              Time triggerTime,
              const std::string& triggerReason = "rssi")
{
    if (!g_hybridContext.enabled || staIndex >= hotspotConfig.staNodes.GetN())
    {
        return;
    }

    Ptr<Node> ue = hotspotConfig.staNodes.Get(staIndex);
    Ptr<Ipv4> ipv4 = ue->GetObject<Ipv4>();
    Ipv4StaticRoutingHelper helper;
    Ptr<Ipv4StaticRouting> rt = helper.GetStaticRouting(ipv4);

    Ptr<NetDevice> lteDev = lteConfig.ueDevices.Get(staIndex);
    uint32_t lteIf = ipv4->GetInterfaceForDevice(lteDev);

    // Make-before-break: install LTE route first, then remove any stale routes on other
    // interfaces. There is never a moment where serviceIp has no active route.
    rt->AddHostRouteTo(lteConfig.serviceIp, lteConfig.ueGateway, lteIf);
    RemoveHostRoutesExcept(rt, lteConfig.serviceIp, lteIf);
    DumpStaRoutingTable(staIndex, rt, "after wifi->lte host-route install");

    HybridUeState& st = g_hybridContext.ueStates[staIndex];
    st.currentPath = USE_LTE;
    st.lastSwitch = Simulator::Now();
    // Clear PDR window so old path's RX history does not contaminate the new path's PDR.
    st.txTimes.clear();
    st.rxTimes.clear();
    st.pdrWindow = 1.0;
    st.lastWindowTxPackets = 0;
    st.lastWindowRxPackets = 0;
    CreateSwitchEvent(staIndex, "wifi", "lte", triggerTime, st.rssiAvgDbm, lteConfig.serviceIp, triggerReason);
}

static void
SwitchUeToWifi(uint32_t staIndex,
               const HotspotConfig& hotspotConfig,
               Ipv4Address serviceIp,
               const std::string& fromPath,
               Time triggerTime,
               const std::string& triggerReason = "rssi+pdr")
{
    if (!g_hybridContext.enabled || staIndex >= hotspotConfig.staNodes.GetN())
    {
        return;
    }

    Ptr<Node> ue = hotspotConfig.staNodes.Get(staIndex);
    Ptr<Ipv4> ipv4 = ue->GetObject<Ipv4>();
    Ipv4StaticRoutingHelper helper;
    Ptr<Ipv4StaticRouting> rt = helper.GetStaticRouting(ipv4);

    // Resolve the WiFi next-hop BEFORE touching the routing table.
    // If no valid WiFi route can be found, keep the cellular route active
    // rather than leaving the service IP unreachable on any path.
    uint32_t apNodeIndexForRoute = std::numeric_limits<uint32_t>::max();
    Ipv4Address apHotspotIp{"0.0.0.0"};
    uint32_t staIf = std::numeric_limits<uint32_t>::max();
    if (staIndex < hotspotConfig.currentStaApIndex.size())
    {
        uint32_t apNodeIndex = hotspotConfig.currentStaApIndex[staIndex];
        if (apNodeIndex != std::numeric_limits<uint32_t>::max() &&
            apNodeIndex < hotspotConfig.apInterfaces.GetN())
        {
            apNodeIndexForRoute = apNodeIndex;
            apHotspotIp = hotspotConfig.apInterfaces.GetAddress(apNodeIndex);
            Ptr<NetDevice> staDev = hotspotConfig.staDevices.Get(staIndex);
            staIf = ipv4->GetInterfaceForDevice(staDev);
        }
    }

    // Option B: if association isn't available, route via best recent AP RSSI.
    if (apNodeIndexForRoute == std::numeric_limits<uint32_t>::max())
    {
        HybridUeState& st = g_hybridContext.ueStates[staIndex];
        if (st.apRssiAvgDbm.size() == hotspotConfig.apInterfaces.GetN() &&
            st.apRssiLastUpdate.size() == hotspotConfig.apInterfaces.GetN())
        {
            const Time now = Simulator::Now();
            double bestRssi = -1000.0;
            uint32_t bestAp = std::numeric_limits<uint32_t>::max();
            for (uint32_t ap = 0; ap < st.apRssiAvgDbm.size(); ++ap)
            {
                if ((now - st.apRssiLastUpdate[ap]) > g_apRssiFreshness)
                {
                    continue;
                }
                if (st.apRssiAvgDbm[ap] > bestRssi)
                {
                    bestRssi = st.apRssiAvgDbm[ap];
                    bestAp = ap;
                }
            }

            if (bestAp != std::numeric_limits<uint32_t>::max())
            {
                apNodeIndexForRoute = bestAp;
                apHotspotIp = hotspotConfig.apInterfaces.GetAddress(bestAp);
                Ptr<NetDevice> staDev = hotspotConfig.staDevices.Get(staIndex);
                staIf = ipv4->GetInterfaceForDevice(staDev);
                if (g_switchDebug)
                {
                    std::cout << Simulator::Now().GetSeconds()
                              << "s [switch-debug] STA " << staIndex
                              << " fallback AP selection: bestAp=" << bestAp
                              << " bestRssiDbm=" << bestRssi
                              << " freshness_s=" << g_apRssiFreshness.GetSeconds()
                              << std::endl;
                }
            }
        }
    }
    // Abort if no valid WiFi next-hop was resolved — keep cellular route intact.
    if (staIf == std::numeric_limits<uint32_t>::max())
    {
        if (g_switchDebug)
        {
            std::cout << Simulator::Now().GetSeconds() << "s [switch-debug] STA " << staIndex
                      << " " << fromPath << "->wifi aborted: no valid WiFi next-hop"
                      << " (no assoc and no fresh RSSI sample)"
                      << std::endl;
        }
        return;
    }

    // Make-before-break: install WiFi route first, then remove stale routes on other interfaces.
    rt->AddHostRouteTo(serviceIp, apHotspotIp, staIf);
    RemoveHostRoutesExcept(rt, serviceIp, staIf);

    // Refresh gateway downlink host route immediately.
    // HandleStaAssociation only fires on MAC-level re-association which may
    // not have happened yet (or may point to a different AP if the STA roamed
    // while on cellular). Calling UpdateStaRouting here ensures the gateway
    // sends downlink packets to the correct mesh node at the moment of the switch.
    UpdateStaRouting(staIndex, apNodeIndexForRoute);

    if (g_switchDebug)
    {
        std::cout << Simulator::Now().GetSeconds() << "s [switch-debug] STA " << staIndex
                  << " " << fromPath << "->wifi"
                  << " triggerReason=" << triggerReason
                  << " apNodeIndex=" << apNodeIndexForRoute
                  << " nextHop(apHotspotIp)=" << apHotspotIp
                  << " staIf=" << staIf
                  << " serviceIp=" << serviceIp
                  << std::endl;
    }
    DumpStaRoutingTable(staIndex, rt, "after ->wifi host-route install");

    HybridUeState& st = g_hybridContext.ueStates[staIndex];
    st.currentPath = USE_WIFI;
    st.lastSwitch = Simulator::Now();
    // Clear PDR window so old path's RX history does not contaminate the new path's PDR.
    st.txTimes.clear();
    st.rxTimes.clear();
    st.pdrWindow = 1.0;
    st.lastWindowTxPackets = 0;
    st.lastWindowRxPackets = 0;
    CreateSwitchEvent(staIndex, fromPath, "wifi", triggerTime, st.rssiAvgDbm, serviceIp, triggerReason);
}

static void
SwitchUeToNr(uint32_t staIndex,
             const HotspotConfig& hotspotConfig,
             const NrHybridConfig& nrConfig,
             Time triggerTime,
             const std::string& triggerReason = "rssi")
{
    if (!g_hybridContext.enabled || staIndex >= hotspotConfig.staNodes.GetN())
    {
        return;
    }

    Ptr<Node> ue = hotspotConfig.staNodes.Get(staIndex);
    Ptr<Ipv4> ipv4 = ue->GetObject<Ipv4>();
    Ipv4StaticRoutingHelper helper;
    Ptr<Ipv4StaticRouting> rt = helper.GetStaticRouting(ipv4);

    Ptr<NetDevice> nrDev = nrConfig.ueDevices.Get(staIndex);
    uint32_t nrIf = ipv4->GetInterfaceForDevice(nrDev);

    // Make-before-break: install NR route first, then remove any stale routes on other interfaces.
    rt->AddHostRouteTo(nrConfig.serviceIp, nrConfig.ueGateway, nrIf);
    RemoveHostRoutesExcept(rt, nrConfig.serviceIp, nrIf);
    DumpStaRoutingTable(staIndex, rt, "after wifi->nr host-route install");

    HybridUeState& st = g_hybridContext.ueStates[staIndex];
    st.currentPath = USE_NR;
    st.lastSwitch = Simulator::Now();
    // Clear PDR window so old path's RX history does not contaminate the new path's PDR.
    st.txTimes.clear();
    st.rxTimes.clear();
    st.pdrWindow = 1.0;
    st.lastWindowTxPackets = 0;
    st.lastWindowRxPackets = 0;
    CreateSwitchEvent(staIndex, "wifi", "nr", triggerTime, st.rssiAvgDbm, nrConfig.serviceIp, triggerReason);
}

static void
CheckHybridSwitchingLte(const HotspotConfig& hotspotConfig,
                        const LteHybridConfig& lteConfig,
                        Time interval,
                        double rssiThresholdDbm,
                        double rssiHysteresisDb)
{
    if (!g_hybridContext.enabled)
    {
        return;
    }

    for (uint32_t i = 0; i < hotspotConfig.staNodes.GetN(); ++i)
    {
        HybridUeState& st = g_hybridContext.ueStates[i];
        if (!st.rssiInitialized)
        {
            continue;
        }

        if (Simulator::Now() - st.lastSwitch < g_hybridContext.minSwitchInterval)
        {
            continue;
        }

        if (st.currentPath == USE_WIFI)
        {
            // Primary trigger: RSSI below threshold.
            // Secondary trigger: PDR drops below pdrThreshold (catches interference/congestion).
            // Either condition alone is sufficient to switch to cellular.
            // Guard: hasEverHadService prevents premature switches before service traffic has
            // established. hasWindowTx is NOT required here — if WiFi is broken after a failed
            // LTE->WiFi return, the cleared PDR window will have TX=0 and the node would be stuck
            // permanently. RSSI is radio-based and valid without TX; PDR already guards itself
            // internally with its own TX count check.
            bool rssiBad = (st.rssiAvgDbm < rssiThresholdDbm);
            bool pdrBad = (st.pdrWindow < g_hybridContext.pdrThreshold) &&
                          (st.lastWindowTxPackets >= g_hybridContext.minTxPacketsForSwitch);
            // If WiFi beacons stop arriving (STA out of range), treat coverage as lost.
            bool rssiStale = st.rssiInitialized &&
                             (Simulator::Now() - st.lastRssiSampleTime > Seconds(3.0));
            bool hasEverHadService = st.hasLastServiceRx;

            if ((rssiBad || pdrBad || rssiStale) && hasEverHadService)
            {
                std::string reason = rssiStale ? "rssi_stale"
                                   : (rssiBad && pdrBad) ? "rssi+pdr"
                                   : (rssiBad ? "rssi" : "pdr");
                const Time triggerTime = Simulator::Now();
                SwitchUeToLte(i, hotspotConfig, lteConfig, triggerTime, reason);
            }
            st.wifiReturnGoodCount = 0;
        }
        else
        {
            // Return to WiFi based on RSSI alone (plus the consecutive-good-checks guard).
            // pdrGood/hasRecentService/hasWindowTx all measure the CURRENT cellular path —
            // requiring them here creates a deadlock when cellular itself degrades: the STA
            // can never return to WiFi precisely in the scenario where it should.
            bool rssiGood = (st.rssiAvgDbm > (rssiThresholdDbm + rssiHysteresisDb));
            bool hasAssoc = true;
            if (g_hybridContext.requireAssocForWifiReturn && i < hotspotConfig.currentStaApIndex.size())
            {
                hasAssoc = (hotspotConfig.currentStaApIndex[i] != std::numeric_limits<uint32_t>::max());
            }
            if (!hasAssoc && g_hybridContext.kickAssocOnWifiReturn)
            {
                Ptr<WifiNetDevice> staWifiDev = DynamicCast<WifiNetDevice>(hotspotConfig.staDevices.Get(i));
                if (staWifiDev)
                {
                    Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac>(staWifiDev->GetMac());
                    if (staMac)
                    {
                        staMac->SetAttribute("ActiveProbing", BooleanValue(true));
                        if (g_switchDebug)
                        {
                            std::cout << Simulator::Now().GetSeconds()
                                      << "s [switch-debug] STA " << i
                                      << " association kick: ActiveProbing=true (return-to-wifi desired but no assoc)"
                                      << std::endl;
                        }
                    }
                }
            }
            if (rssiGood && hasAssoc)
            {
                st.wifiReturnGoodCount++;
                if (st.wifiReturnGoodCount >= g_hybridContext.wifiReturnGoodChecks)
                {
                    const Time triggerTime = Simulator::Now();
                    SwitchUeToWifi(i, hotspotConfig, lteConfig.serviceIp, "lte", triggerTime, "rssi");
                    st.wifiReturnGoodCount = 0;
                }
            }
            else
            {
                st.wifiReturnGoodCount = 0;
            }
        }
    }

    Simulator::Schedule(interval, &CheckHybridSwitchingLte,
                        hotspotConfig, lteConfig, interval, rssiThresholdDbm, rssiHysteresisDb);
}

static void
CheckHybridSwitchingNr(const HotspotConfig& hotspotConfig,
                       const NrHybridConfig& nrConfig,
                       Time interval,
                       double rssiThresholdDbm,
                       double rssiHysteresisDb)
{
    if (!g_hybridContext.enabled)
    {
        return;
    }

    for (uint32_t i = 0; i < hotspotConfig.staNodes.GetN(); ++i)
    {
        HybridUeState& st = g_hybridContext.ueStates[i];
        if (!st.rssiInitialized)
        {
            continue;
        }

        if (Simulator::Now() - st.lastSwitch < g_hybridContext.minSwitchInterval)
        {
            continue;
        }

        if (st.currentPath == USE_WIFI)
        {
            // Primary trigger: RSSI below threshold.
            // Secondary trigger: PDR drops below pdrThreshold (catches interference/congestion).
            // Either condition alone is sufficient to switch to cellular.
            // Guard: hasEverHadService prevents premature switches before service traffic has
            // established. hasWindowTx is NOT required here — if WiFi is broken after a failed
            // LTE->WiFi return, the cleared PDR window will have TX=0 and the node would be stuck
            // permanently. RSSI is radio-based and valid without TX; PDR already guards itself
            // internally with its own TX count check.
            bool rssiBad = (st.rssiAvgDbm < rssiThresholdDbm);
            bool pdrBad = (st.pdrWindow < g_hybridContext.pdrThreshold) &&
                          (st.lastWindowTxPackets >= g_hybridContext.minTxPacketsForSwitch);
            // If WiFi beacons stop arriving (STA out of range), treat coverage as lost.
            bool rssiStale = st.rssiInitialized &&
                             (Simulator::Now() - st.lastRssiSampleTime > Seconds(3.0));
            bool hasEverHadService = st.hasLastServiceRx;

            if ((rssiBad || pdrBad || rssiStale) && hasEverHadService)
            {
                std::string reason = rssiStale ? "rssi_stale"
                                   : (rssiBad && pdrBad) ? "rssi+pdr"
                                   : (rssiBad ? "rssi" : "pdr");
                const Time triggerTime = Simulator::Now();
                SwitchUeToNr(i, hotspotConfig, nrConfig, triggerTime, reason);
            }
            st.wifiReturnGoodCount = 0;
        }
        else
        {
            // Return to WiFi based on RSSI alone (plus the consecutive-good-checks guard).
            // pdrGood/hasRecentService/hasWindowTx all measure the CURRENT cellular path —
            // requiring them here creates a deadlock when cellular itself degrades: the STA
            // can never return to WiFi precisely in the scenario where it should.
            bool rssiGood = (st.rssiAvgDbm > (rssiThresholdDbm + rssiHysteresisDb));
            bool hasAssoc = true;
            if (g_hybridContext.requireAssocForWifiReturn && i < hotspotConfig.currentStaApIndex.size())
            {
                hasAssoc = (hotspotConfig.currentStaApIndex[i] != std::numeric_limits<uint32_t>::max());
            }
            if (!hasAssoc && g_hybridContext.kickAssocOnWifiReturn)
            {
                Ptr<WifiNetDevice> staWifiDev = DynamicCast<WifiNetDevice>(hotspotConfig.staDevices.Get(i));
                if (staWifiDev)
                {
                    Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac>(staWifiDev->GetMac());
                    if (staMac)
                    {
                        staMac->SetAttribute("ActiveProbing", BooleanValue(true));
                        if (g_switchDebug)
                        {
                            std::cout << Simulator::Now().GetSeconds()
                                      << "s [switch-debug] STA " << i
                                      << " association kick: ActiveProbing=true (return-to-wifi desired but no assoc)"
                                      << std::endl;
                        }
                    }
                }
            }
            if (rssiGood && hasAssoc)
            {
                st.wifiReturnGoodCount++;
                if (st.wifiReturnGoodCount >= g_hybridContext.wifiReturnGoodChecks)
                {
                    const Time triggerTime = Simulator::Now();
                    SwitchUeToWifi(i, hotspotConfig, nrConfig.serviceIp, "nr", triggerTime, "rssi");
                    st.wifiReturnGoodCount = 0;
                }
            }
            else
            {
                st.wifiReturnGoodCount = 0;
            }
        }
    }

    Simulator::Schedule(interval, &CheckHybridSwitchingNr,
                        hotspotConfig, nrConfig, interval, rssiThresholdDbm, rssiHysteresisDb);
}

int main(int argc, char* argv[])
{
    // Enable packet metadata for NetAnim
    PacketMetadata::Enable();
    Packet::EnablePrinting();

    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(10 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(10 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));

    uint32_t nNodes = 4;           // Number of mesh nodes (four-AP layout)
    uint32_t gridWidth = 0;        // Grid width (unused for custom layout)
    const uint32_t kDefaultPacketSize = 1400;
    uint32_t packetSize = kDefaultPacketSize;    // Packet size in bytes
    double nodeSpacing = 150.0;    // Approximate spacing used for fallback positioning (meters)
    uint32_t tcpPacketSize = kDefaultPacketSize; // TCP packet size in bytes (legacy alias for packetSize)
    double simTime = 90.0;         // Simulation time (seconds)
    bool enableHotspot = true;     // Enable hotspot (AP + STA) feature
    uint32_t apNodeIndex = 3;      // Which mesh node acts as AP (0-based)
    uint32_t numStaNodes = 1;     // Single STA client (was 10)
    double meshApHeight = 1.5;     // Mesh AP height (meters) - for height optimization tests
    double staHeight = 5.0;        // STA node height (meters) - for vertical spacing tests
    double staSpeedMin = 5.0;      // STA mean speed lower bound (m/s)
    double staSpeedMax = 5.0;      // STA mean speed upper bound (m/s)
    uint32_t rngSeed = 6;          // RNG seed for reproducible runs
    uint32_t meshConfig = 1;       // Mesh AP device configuration (0=Default, 1=TP-Link EAP225, 2=Netgear Orbi 960, 3=Asus ZenWiFi XT8)
    uint32_t uploadBytes = 0;    // 0 = unlimited (runs for full simTime)
    uint32_t downloadBytes = 0;  // 0 = unlimited (runs for full simTime)
    uint32_t voipBytes = 0;      // 0 = unlimited (runs for full simTime)
    std::string hotspotBand = "5g"; // Hotspot band selector (5g or 2g)
    // Root output directory for Wi-Fi hybrid runs (in repo root)
    std::string outputDir = "Wifi_hybrid_outputs";
    std::string cellularMode = "lte"; // cellular fallback mode: lte (default) or nr
    double flowScale = 1.0;
    bool enableSwitching = true;
    double rssiThresholdDbm = -80.0;
    double rssiHysteresisDb = 3.0;
    double pdrThreshold = 0.9;
    double wifiReturnPdrThreshold = 0.90;
    double switchIntervalSec = 0.5;
    double minSwitchIntervalSec = 5.0;
    double switchTimeoutSec = 5.0;
    double activityWindowSec = 2.0;
    uint32_t minTxPacketsForSwitch = 1;
    uint32_t wifiReturnGoodChecks = 2;
    double pdrWindowSec = 1.0;
    double pdrUpdateIntervalSec = 1.0;
    bool enablePdrLog = true;
    bool enableSwitchDebug = false;
    double apRssiFreshnessSec = 1.0;
    bool requireAssocForWifiReturn = false;
    bool kickAssocOnWifiReturn = true;
    std::string mobilityModel = "gaussmarkov"; // gaussmarkov (default) or waypoint
    std::string robotType = "patrol";          // waypoint robot archetype: patrol, transport, or work

    CommandLine cmd;
    cmd.AddValue("nNodes", "Number of mesh nodes", nNodes);
    cmd.AddValue("gridWidth", "Grid width (for NxN grid)", gridWidth);
    cmd.AddValue("packetSize", "Size of application packet in bytes", packetSize);
    cmd.AddValue("nodeSpacing", "Distance between adjacent nodes (meters)", nodeSpacing);
    cmd.AddValue("tcpPacketSize", "TCP packet size in bytes", tcpPacketSize);
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue("enableHotspot", "Enable hotspot (AP + STA) feature", enableHotspot);
    cmd.AddValue("apNodeIndex", "Which mesh node acts as AP (0-3)", apNodeIndex);
    cmd.AddValue("numStaNodes", "Number of STA clients", numStaNodes);
    cmd.AddValue("meshApHeight", "Mesh AP height in meters (1.5, 10, 15)", meshApHeight);
    cmd.AddValue("staHeight", "STA node height in meters (0-30)", staHeight);
    cmd.AddValue("staSpeedMin", "STA mean speed minimum in m/s", staSpeedMin);
    cmd.AddValue("staSpeedMax", "STA mean speed maximum in m/s", staSpeedMax);
    cmd.AddValue("meshConfig", "Mesh AP device (0=Default 802.11g, 1=TP-Link EAP225, 2=Netgear Orbi 960, 3=Asus ZenWiFi XT8)", meshConfig);
    cmd.AddValue("uploadBytes", "Total bytes for each STA TCP upload flow", uploadBytes);
    cmd.AddValue("downloadBytes", "Total bytes for each STA TCP download flow", downloadBytes);
    cmd.AddValue("voipBytes", "Total bytes for each VoIP OnOff flow", voipBytes);
    cmd.AddValue("hotspotBand", "Hotspot band for STA AP radios (5g or 2g)", hotspotBand);
    cmd.AddValue("outputDir", "Directory where run artifacts (metrics, flowmon, config) are stored", outputDir);
    cmd.AddValue("cellularMode", "Cellular fallback mode (lte or nr)", cellularMode);
    cmd.AddValue("flowScale",
                 "Multiplier applied to upload/download byte budgets (1=1MB default)", flowScale);
    cmd.AddValue("rngSeed", "RNG seed for reproducible runs", rngSeed);
    cmd.AddValue("enableSwitching", "Enable RSSI-based WiFi<->cellular switching for service traffic", enableSwitching);
    cmd.AddValue("rssiThresholdDbm", "WiFi RSSI threshold (dBm) below which to switch to cellular", rssiThresholdDbm);
    cmd.AddValue("rssiHysteresisDb", "RSSI hysteresis (dB) above threshold to switch back to WiFi", rssiHysteresisDb);
    cmd.AddValue("pdrThreshold", "Per-STA PDR threshold (0..1) below which switching can trigger", pdrThreshold);
    cmd.AddValue("wifiReturnPdrThreshold", "Per-STA PDR threshold (0..1) required for cellular->WiFi return", wifiReturnPdrThreshold);
    cmd.AddValue("switchIntervalSec", "Switch controller interval (seconds)", switchIntervalSec);
    cmd.AddValue("minSwitchIntervalSec", "Minimum dwell time between path switches per STA (seconds)", minSwitchIntervalSec);
    cmd.AddValue("switchTimeoutSec", "Timeout for pending switch recovery marking (seconds)", switchTimeoutSec);
    cmd.AddValue("activityWindowSec", "Recent service RX window required to allow WiFi->cell switching (seconds)", activityWindowSec);
    cmd.AddValue("minTxPacketsForSwitch", "Minimum per-window TX packets required before switching", minTxPacketsForSwitch);
    cmd.AddValue("wifiReturnGoodChecks", "Consecutive good RSSI checks required before cellular->WiFi return", wifiReturnGoodChecks);
    cmd.AddValue("pdrWindowSec", "Sliding window duration for PDR calculation (seconds)", pdrWindowSec);
    cmd.AddValue("pdrUpdateIntervalSec", "How often to update/log window PDR (seconds)", pdrUpdateIntervalSec);
    cmd.AddValue("enablePdrLog", "Write window PDR time-series CSV during simulation", enablePdrLog);
    cmd.AddValue("enableSwitchDebug", "Print per-switch routing/next-hop debug to stdout", enableSwitchDebug);
    cmd.AddValue("apRssiFreshnessSec", "Max age (seconds) of AP RSSI samples for fallback AP selection", apRssiFreshnessSec);
    cmd.AddValue("requireAssocForWifiReturn", "Option A: only allow cellular->WiFi return when STA is associated to an AP", requireAssocForWifiReturn);
    cmd.AddValue("kickAssocOnWifiReturn", "When WiFi return desired but no assoc, restart STA scanning (ActiveProbing=true)", kickAssocOnWifiReturn);
    cmd.AddValue("mobilityModel", "STA mobility model: gaussmarkov (default) or waypoint", mobilityModel);
    cmd.AddValue("robotType", "Waypoint robot archetype when mobilityModel=waypoint: patrol, transport, or work", robotType);
    cmd.Parse(argc, argv);

    // Automatically nest output under seed subfolder when using the default output directory.
    if (outputDir == "Wifi_hybrid_outputs")
    {
        outputDir = "Wifi_hybrid_outputs/seed" + std::to_string(rngSeed);
    }

    RngSeedManager::SetSeed(rngSeed);
    RngSeedManager::SetRun(rngSeed);

    if (packetSize == kDefaultPacketSize && tcpPacketSize != kDefaultPacketSize)
    {
        packetSize = tcpPacketSize;
    }
    else
    {
        tcpPacketSize = packetSize;
    }

    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(packetSize));
    
    auto scaleBytes = [flowScale](uint32_t value) -> uint32_t {
        if (value == 0) return 0; // 0 means unlimited — preserve as-is
        return static_cast<uint32_t>(std::max(1.0, std::round(static_cast<double>(value) * flowScale)));
    };
    uploadBytes = scaleBytes(uploadBytes);
    downloadBytes = scaleBytes(downloadBytes);

    std::string hotspotBandNormalized = hotspotBand;
    std::transform(hotspotBandNormalized.begin(),
                   hotspotBandNormalized.end(),
                   hotspotBandNormalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (hotspotBandNormalized != "2g" && hotspotBandNormalized != "5g")
    {
        NS_LOG_WARN("Invalid hotspotBand value '" << hotspotBand
                     << "'. Supported values are '5g' or '2g'. Defaulting to 5g.");
        hotspotBandNormalized = "5g";
    }
    hotspotBand = hotspotBandNormalized;
    if (staSpeedMin < 0.0 || staSpeedMax < 0.0 || staSpeedMin > staSpeedMax)
    {
        NS_LOG_WARN("Invalid STA speed range [" << staSpeedMin << ", " << staSpeedMax
                     << "], resetting to defaults [1.0, 3.0] m/s.");
        staSpeedMin = 5.0;
        staSpeedMax = 5.0;
    }
    if (pdrThreshold < 0.0 || pdrThreshold > 1.0)
    {
        NS_LOG_WARN("Invalid pdrThreshold value " << pdrThreshold
                    << ", resetting to default 0.9.");
        pdrThreshold = 0.9;
    }
    if (wifiReturnPdrThreshold < 0.0 || wifiReturnPdrThreshold > 1.0)
    {
        NS_LOG_WARN("Invalid wifiReturnPdrThreshold value " << wifiReturnPdrThreshold
                    << ", resetting to default 0.90.");
        wifiReturnPdrThreshold = 0.90;
    }
    if (minSwitchIntervalSec < 0.0)
    {
        NS_LOG_WARN("Invalid minSwitchIntervalSec value " << minSwitchIntervalSec
                    << ", resetting to default 5.0.");
        minSwitchIntervalSec = 5.0;
    }
    if (switchTimeoutSec <= 0.0)
    {
        NS_LOG_WARN("Invalid switchTimeoutSec value " << switchTimeoutSec
                    << ", resetting to default 5.0.");
        switchTimeoutSec = 5.0;
    }
    if (activityWindowSec <= 0.0)
    {
        NS_LOG_WARN("Invalid activityWindowSec value " << activityWindowSec
                    << ", resetting to default 2.0.");
        activityWindowSec = 2.0;
    }
    if (wifiReturnGoodChecks == 0)
    {
        NS_LOG_WARN("Invalid wifiReturnGoodChecks value 0, resetting to default 2.");
        wifiReturnGoodChecks = 2;
    }

    std::string cellularModeNormalized = cellularMode;
    std::transform(cellularModeNormalized.begin(),
                   cellularModeNormalized.end(),
                   cellularModeNormalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (cellularModeNormalized != "lte" && cellularModeNormalized != "nr")
    {
        NS_LOG_WARN("Invalid cellularMode value '" << cellularMode
                    << "'. Supported values are 'lte' or 'nr'. Defaulting to lte.");
        cellularModeNormalized = "lte";
    }
    cellularMode = cellularModeNormalized;
    bool useNrCellular = (cellularMode == "nr");

    std::string mobilityModelNormalized = mobilityModel;
    std::transform(mobilityModelNormalized.begin(),
                   mobilityModelNormalized.end(),
                   mobilityModelNormalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mobilityModelNormalized != "gaussmarkov" && mobilityModelNormalized != "waypoint")
    {
        NS_LOG_WARN("Invalid mobilityModel value '" << mobilityModel
                    << "'. Supported values are 'gaussmarkov' or 'waypoint'. Defaulting to gaussmarkov.");
        mobilityModelNormalized = "gaussmarkov";
    }
    mobilityModel = mobilityModelNormalized;

    std::string robotTypeNormalized = robotType;
    std::transform(robotTypeNormalized.begin(),
                   robotTypeNormalized.end(),
                   robotTypeNormalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (robotTypeNormalized != "patrol" && robotTypeNormalized != "transport" && robotTypeNormalized != "work")
    {
        NS_LOG_WARN("Invalid robotType value '" << robotType
                    << "'. Supported values are 'patrol', 'transport', or 'work'. Defaulting to patrol.");
        robotTypeNormalized = "patrol";
    }
    robotType = robotTypeNormalized;

    fs::create_directories(fs::path(outputDir));
    
    // Validate mesh config
    if (meshConfig > 3) {
        NS_LOG_WARN("Invalid meshConfig value " << meshConfig << ", using default (0)");
        meshConfig = 0;
    }
    
    // Get selected mesh device configuration
    MeshAPDeviceConfig deviceCfg = GetMeshDeviceConfig(meshConfig);
    
    const std::vector<Vector> layoutPositions = GetFourNodeMeshPositions(meshApHeight);
    if (nNodes != layoutPositions.size())
    {
        NS_LOG_WARN("Overriding nNodes (" << nNodes << ") to match four-node layout ("
                     << layoutPositions.size() << ")");
        nNodes = layoutPositions.size();
    }

    gridWidth = 0;  // indicates custom layout for logging purposes

    // Update AP node index (last node in layout)
    apNodeIndex = nNodes > 0 ? nNodes - 1 : 0;
    
    NS_LOG_UNCOND("  Device: " << deviceCfg.name);
    NS_LOG_UNCOND("  Description: " << deviceCfg.description);
    NS_LOG_UNCOND("  WiFi Standard: " << deviceCfg.wifiStandard);
    NS_LOG_UNCOND("  Data Mode: " << deviceCfg.dataMode);
    NS_LOG_UNCOND("  TX Power (Mesh Backhaul): " << deviceCfg.txPowerStart << " dBm");
    NS_LOG_UNCOND("  TX Power (Hotspot/AP): " << deviceCfg.hotspotTxPower << " dBm");
    NS_LOG_UNCOND("  RX Sensitivity: " << deviceCfg.rxSensitivity << " dBm");
    NS_LOG_UNCOND("  Antenna Gain (RX/TX): " << deviceCfg.rxGain << "/" << deviceCfg.txGain << " dB");
    NS_LOG_UNCOND("  Number of Interfaces: " << deviceCfg.numInterfaces);

    MeshNetworkConfig meshNetConfig = SetupMeshNetwork(nNodes, gridWidth, nodeSpacing, meshApHeight, deviceCfg);

    InternetConfig internetConfig = SetupInternetInfrastructure(meshNetConfig.meshNodes, nodeSpacing, meshApHeight);

    HotspotConfig hotspotConfig;
    if (enableHotspot)
    {
        hotspotConfig = SetupHotspotInfrastructure(meshNetConfig.meshNodes,
                                                   apNodeIndex,
                                                   numStaNodes,
                                                   nodeSpacing,
                                                   staHeight,
                                                   staSpeedMin,
                                                   staSpeedMax,
                                                   deviceCfg,
                                                   hotspotBand,  // Pass device config for hotspot TX power
                                                   simTime,
                                                   mobilityModel,
                                                   robotType);
    }

    // Initialize hybrid controller state (one entry per STA)
    if (enableHotspot && hotspotConfig.staNodes.GetN() > 0)
    {
        g_switchDebug = enableSwitchDebug;
        g_apRssiFreshness = Seconds(apRssiFreshnessSec);
        g_hybridContext.enabled = true;
        g_hybridContext.ueStates.assign(hotspotConfig.staNodes.GetN(), HybridUeState{});
        g_hybridContext.nodeIdToStaIndex = &hotspotConfig.nodeIdToStaIndex;
        g_hybridContext.hotspotConfig = &hotspotConfig;
        g_hybridContext.requireAssocForWifiReturn = requireAssocForWifiReturn;
        g_hybridContext.kickAssocOnWifiReturn = kickAssocOnWifiReturn;
        g_hybridContext.rssiThresholdDbm = rssiThresholdDbm;
        g_hybridContext.pdrThreshold = pdrThreshold;
        g_hybridContext.wifiReturnPdrThreshold = wifiReturnPdrThreshold;
        g_hybridContext.pdrWindowDuration = Seconds(pdrWindowSec);
        g_hybridContext.minSwitchInterval = Seconds(minSwitchIntervalSec);
        g_hybridContext.switchTimeout = Seconds(switchTimeoutSec);
        g_hybridContext.activityWindow = Seconds(activityWindowSec);
        g_hybridContext.minTxPacketsForSwitch = minTxPacketsForSwitch;
        g_hybridContext.wifiReturnGoodChecks = wifiReturnGoodChecks;

        // Open RSSI log file in the chosen output directory
        std::string rssiLogPath = outputDir + "/wifi-hybrid-rssi_log.csv";
        g_rssiLog.open(rssiLogPath, std::ios::out | std::ios::trunc);
        if (g_rssiLog.is_open())
        {
            g_rssiLogOpen = true;
            // Keep the original first 6 columns for backward compatibility, then append
            // cellular fields used for LTE/NR RSRP/SINR logging.
            g_rssiLog
                << "time_s,sta_index,node_id,freq_mhz,inst_rssi_dbm,avg_rssi_dbm,"
                << "rat,cell_id,rnti,cc_or_bwp,rsrp_dbm,sinr_db,path,pos_x,pos_y,pos_z\n";
            std::cout << "Hybrid RSSI log will be written to " << rssiLogPath << std::endl;
        }
        else
        {
            std::cerr << "Warning: could not open RSSI log file " << rssiLogPath << std::endl;
        }

        // Open switch event log
        std::string switchLogPath = outputDir + "/wifi-hybrid-switch_log.csv";
        g_switchLog.open(switchLogPath, std::ios::out | std::ios::trunc);
        if (g_switchLog.is_open())
        {
            g_switchLogOpen = true;
            g_switchLog
                << "switch_id,sta_index,from,to,trigger_time_s,apply_time_s,last_ok_before_s,"
                << "first_rx_after_switch_s,service_interruption_ms,status,rssi_avg_dbm,service_ip,trigger_reason\n";
            std::cout << "Hybrid switch log will be written to " << switchLogPath << std::endl;
        }
        else
        {
            std::cerr << "Warning: could not open switch log file " << switchLogPath << std::endl;
        }

        if (enablePdrLog)
        {
            std::string pdrLogPath = outputDir + "/wifi-hybrid-pdr_window.csv";
            g_pdrLog.open(pdrLogPath, std::ios::out | std::ios::trunc);
            if (g_pdrLog.is_open())
            {
                g_pdrLogOpen = true;
                g_pdrLog << "time_s,sta_index,tx_pkts_window,rx_pkts_window,pdr_window,path\n";
                std::cout << "Hybrid window PDR log will be written to " << pdrLogPath << std::endl;
            }
            else
            {
                std::cerr << "Warning: could not open PDR log file " << pdrLogPath << std::endl;
            }
        }

        std::cout << "Hybrid RSSI switching is " << (enableSwitching ? "ENABLED" : "DISABLED")
                  << " [cellular mode=" << cellularMode << "]"
                  << " (threshold=" << rssiThresholdDbm << " dBm"
                  << ", hysteresis=" << rssiHysteresisDb << " dB"
                  << ", pdrThreshold=" << pdrThreshold
                  << ", wifiReturnPdrThreshold=" << wifiReturnPdrThreshold
                  << ", minSwitchInterval=" << minSwitchIntervalSec << " s"
                  << ", switchTimeout=" << switchTimeoutSec << " s"
                  << ", activityWindow=" << activityWindowSec << " s"
                  << ", minTxPacketsForSwitch=" << minTxPacketsForSwitch
                  << ", wifiReturnGoodChecks=" << wifiReturnGoodChecks
                  << ", interval=" << switchIntervalSec << " s)"
                  << std::endl;

        // Connect WiFi RSSI trace for hybrid quality monitoring
        Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                        MakeCallback(&WifiSnifferRxCallback));
    }

    // Cellular overlay: LTE by default, optional NR if requested.
    LteHybridConfig lteConfig;
    NrHybridConfig nrConfig;
    if (enableHotspot && hotspotConfig.staNodes.GetN() > 0)
    {
        // Field size is 400 x 400 m as per project objective
        // Service IP is the WiFi-side internet server address (8.8.8.x)
        Ipv4Address serviceIp = internetConfig.internetInterfaces.GetAddress(1);
        if (useNrCellular)
        {
            nrConfig = SetupNrHybridOverlay(hotspotConfig.staNodes,
                                            internetConfig.internetNodes.Get(0),
                                            serviceIp,
                                            400.0);
        }
        else
        {
            lteConfig = SetupLteHybridOverlay(hotspotConfig.staNodes,
                                              internetConfig.internetNodes.Get(0),
                                              serviceIp,
                                              400.0);
        }
    }

    // Start periodic per-STA PDR window updates for hybrid control.
    if (enableHotspot && hotspotConfig.staNodes.GetN() > 0)
    {
        Simulator::Schedule(Seconds(pdrUpdateIntervalSec), &UpdatePerStaPdr, Seconds(pdrUpdateIntervalSec));
        Simulator::Schedule(Seconds(switchIntervalSec), &CheckPendingSwitchTimeouts, Seconds(switchIntervalSec));
    }

    // Connect cellular PHY quality traces (RSRP/SINR) to the same CSV log as WiFi RSSI.
    if (g_rssiLogOpen && enableHotspot && hotspotConfig.staNodes.GetN() > 0)
    {
        if (useNrCellular)
        {
            Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/ReportRsrp",
                            MakeCallback(&NrUeRsrpCallback));
            Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlDataSinr",
                            MakeCallback(&NrUeDlDataSinrCallback));
        }
        else
        {
            Config::Connect(
                "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr",
                MakeCallback(&LteUeRsrpSinrCallback));
        }
    }

    ConfigureIPForwarding(meshNetConfig.meshNodes, internetConfig.internetNodes);

    ConfigureStaticRouting(meshNetConfig, internetConfig, hotspotConfig, nNodes, enableHotspot);

    EnableStaRoamingTracing(meshNetConfig, hotspotConfig, enableHotspot);

    SetupApplications(meshNetConfig,
                      internetConfig,
                      hotspotConfig,
                      simTime,
                      enableHotspot,
                      packetSize,
                      uploadBytes,
                      downloadBytes,
                      voipBytes,
                      flowScale);

    // Start periodic RSSI-based switching after initial association/traffic startup.
    if (enableSwitching && enableHotspot && hotspotConfig.staNodes.GetN() > 0)
    {
        if (useNrCellular)
        {
            Simulator::Schedule(Seconds(5.0),
                                &CheckHybridSwitchingNr,
                                hotspotConfig,
                                nrConfig,
                                Seconds(switchIntervalSec),
                                rssiThresholdDbm,
                                rssiHysteresisDb);
        }
        else
        {
            Simulator::Schedule(Seconds(5.0),
                                &CheckHybridSwitchingLte,
                                hotspotConfig,
                                lteConfig,
                                Seconds(switchIntervalSec),
                                rssiThresholdDbm,
                                rssiHysteresisDb);
        }
    }

    
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    
    // Save configuration JSON for analysis scripts
    SaveConfigurationJSON(nNodes, gridWidth, numStaNodes, packetSize, nodeSpacing, meshConfig, outputDir,
                           mobilityModel, robotType);
    
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Hybrid Wi-Fi FlowMonitor XML with _data suffix
    std::string flowmonXmlPath = outputDir + "/wifi-hybrid-flowmon_data.xml";
    SaveFlowMonitorResults(monitor, flowmon, flowmonXmlPath);

    if (enableHotspot)
    {
        std::cout << "\nFinal STA -> AP associations\n";
        std::cout << "----------------------------------------\n";
        for (uint32_t i = 0; i < hotspotConfig.currentStaApIndex.size(); ++i)
        {
            uint32_t apIdx = hotspotConfig.currentStaApIndex[i];
            Ipv4Address staIp = hotspotConfig.staInterfaces.GetAddress(i);
            if (apIdx != std::numeric_limits<uint32_t>::max())
            {
                std::cout << "STA " << i << " (" << staIp << ") attached to mesh node "
                          << apIdx << " (AP BSSID ";
                bool printed = false;
                for (const auto& entry : hotspotConfig.apBssidToMeshIndex)
                {
                    if (entry.second == apIdx)
                    {
                        std::cout << entry.first;
                        printed = true;
                        break;
                    }
                }
                if (!printed)
                {
                    std::cout << "unknown";
                }
                std::cout << ")\n";
            }
            else
            {
                std::cout << "STA " << i << " (" << staIp << ") no active association\n";
            }
        }
        std::cout << "----------------------------------------\n\n";
    }

    Simulator::Destroy();

    // Ensure KPI logs are flushed before running the parser.
    if (g_rssiLogOpen)
    {
        g_rssiLog.flush();
        g_rssiLog.close();
        g_rssiLogOpen = false;
    }
    if (g_switchLogOpen)
    {
        for (auto& ev : g_switchEvents)
        {
            if (ev.status == "pending")
            {
                ev.status = "timeout";
                // No RX observed after switch until simulation end.
                // Record a lower-bound interruption duration up to sim stop.
                double baselineS = (ev.lastOkBeforeS >= 0.0) ? ev.lastOkBeforeS : ev.applyTimeS;
                ev.serviceInterruptionMs = std::max(0.0, (simTime - baselineS) * 1000.0);
            }
            g_switchLog << ev.switchId << ","
                        << ev.staIndex << ","
                        << ev.from << ","
                        << ev.to << ","
                        << ev.triggerTimeS << ","
                        << ev.applyTimeS << ","
                        << ev.lastOkBeforeS << ","
                        << ev.firstRxAfterSwitchS << ","
                        << ev.serviceInterruptionMs << ","
                        << ev.status << ","
                        << ev.rssiAvgDbm << ","
                        << ev.serviceIp << ","
                        << ev.triggerReason << "\n";
        }
        g_switchLog.flush();
        g_switchLog.close();
        g_switchLogOpen = false;
    }
    if (g_pdrLogOpen)
    {
        g_pdrLog.flush();
        g_pdrLog.close();
        g_pdrLogOpen = false;
    }

    // Run local Wi-Fi FlowMonitor parser with current folder structure
    std::ostringstream parseCmd;
    parseCmd << "python3 examples/my-scenarios/parse_wifi_flowmon.py "
             << flowmonXmlPath
             << " --sim-time=" << simTime
             << " --md " << outputDir << "/wifi-hybrid-metrics_data.md";

    std::cout << "Running FlowMonitor parser..." << std::endl;
    int parseStatus = std::system(parseCmd.str().c_str());
    if (parseStatus != 0)
    {
        std::cerr << "FlowMonitor parser exited with status " << parseStatus << std::endl;
    }

    return 0;
}


