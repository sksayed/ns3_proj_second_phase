/*
 * Hybrid WiFi + LTE / NR scenario skeleton.
 *
 * - One set of UE nodes, each with:
 *   - WiFi STA interface (7.0.0.x)
 *   - LTE or NR UE interface (10.0.0.x for LTE, 11.0.0.x for NR)
 * - Single InternetStackHelper per UE.
 * - Shared 3D Gauss–Markov mobility, buildings, HybridBuildingsPropagationLossModel.
 * - CLI arguments to control key experiment parameters.
 *
 * NOTE: Handover / routing switching logic is intentionally not implemented yet.
 */

#include "ns3/applications-module.h"
#include "ns3/buildings-helper.h"
#include "ns3/buildings-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/hybrid-buildings-propagation-loss-model.h"
#include "ns3/internet-module.h"
#include "ns3/lte-helper.h"
#include "ns3/point-to-point-epc-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/ssid.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-helper.h"

#include "ns3/ideal-beamforming-algorithm.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace ns3;
namespace fs = std::filesystem;

NS_LOG_COMPONENT_DEFINE("WifiHybridScenario");

// Common environment constants
static const double kFieldSize = 400.0;
static const double kMinHeight = 0.0;
static const double kMaxHeight = 30.0;

// WiFi + cellular subnets
static const char* kWifiSubnet   = "7.0.1.0";
static const char* kWifiMask     = "255.255.255.0";
static const char* kLteSubnet    = "10.0.0.0"; // LTE UE address range (via EPC)
static const char* kNrSubnet     = "11.0.0.0"; // Used implicitly by NR EPC helpers

// ---------------------------------------------------------------------------//
// WiFi RSSI monitoring (for future switching logic)
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
    // For now, just log RSSI; later this can feed per-UE quality state
    double rssiDbm = signalNoise.signal;
    double noiseDbm = signalNoise.noise;
    NS_LOG_DEBUG("WifiSnifferRx: ctx=" << context
                                       << " freq=" << channelFreqMhz << " MHz"
                                       << " RSSI=" << rssiDbm << " dBm"
                                       << " noise=" << noiseDbm << " dBm");
}

// Mesh AP device configuration (copied from wifi_mesh_simulation_file.cc)
struct MeshAPDeviceConfig
{
    std::string name;
    std::string description;

    double txPowerStart;
    double txPowerEnd;
    double hotspotTxPower;
    double rxSensitivity;
    double rxGain;
    double txGain;

    WifiStandard wifiStandard;
    std::string dataMode;
    uint32_t numInterfaces;
    double meshRange;

    MeshAPDeviceConfig(std::string n,
                       std::string desc,
                       double txStart,
                       double txEnd,
                       double hotspotTx,
                       double rxSens,
                       double rxG,
                       double txG,
                       WifiStandard standard,
                       std::string mode,
                       uint32_t interfaces,
                       double range)
        : name(n),
          description(desc),
          txPowerStart(txStart),
          txPowerEnd(txEnd),
          hotspotTxPower(hotspotTx),
          rxSensitivity(rxSens),
          rxGain(rxG),
          txGain(txG),
          wifiStandard(standard),
          dataMode(mode),
          numInterfaces(interfaces),
          meshRange(range)
    {}
};

MeshAPDeviceConfig
GetMeshDeviceConfig(uint32_t configId)
{
    switch (configId)
    {
    case 1:
        return MeshAPDeviceConfig("TP-Link EAP225-Outdoor",
                                  "802.11n mesh @ 2.4GHz backhaul, 5GHz hotspot",
                                  23.0,
                                  23.0,
                                  22.0,
                                  -90.0,
                                  5.0,
                                  5.0,
                                  WIFI_STANDARD_80211n,
                                  "HtMcs7",
                                  1,
                                  300.0);
    case 2:
        return MeshAPDeviceConfig("Netgear Orbi 960",
                                  "Tri-band WiFi 6E mesh with dedicated backhaul",
                                  23.0,
                                  23.0,
                                  28.0,
                                  -92.0,
                                  6.0,
                                  6.0,
                                  WIFI_STANDARD_80211ax,
                                  "HeMcs11",
                                  1,
                                  350.0);
    case 3:
        return MeshAPDeviceConfig("Asus ZenWiFi XT8",
                                  "Dual-band WiFi 6 mesh, backhaul + hotspot",
                                  20.0,
                                  20.0,
                                  24.0,
                                  -90.0,
                                  4.0,
                                  4.0,
                                  WIFI_STANDARD_80211ax,
                                  "HeMcs9",
                                  1,
                                  250.0);
    default:
        return MeshAPDeviceConfig("Generic 802.11n AP",
                                  "Default 2.4GHz mesh backhaul with 5GHz hotspot",
                                  20.0,
                                  20.0,
                                  20.0,
                                  -88.0,
                                  3.0,
                                  3.0,
                                  WIFI_STANDARD_80211n,
                                  "HtMcs7",
                                  1,
                                  250.0);
    }
}

// ---------------------------------------------------------------------------//
// Mobility and buildings helpers
// ---------------------------------------------------------------------------//

void
ConfigureUeMobility(NodeContainer& ueNodes,
                    double field,
                    double minHeight,
                    double maxHeight)
{
    // 3D Gauss–Markov mobility over the worksite volume.
    double minX = 0.0;
    double maxX = field;
    double minY = 0.0;
    double maxY = field;

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();

    Ptr<UniformRandomVariable> xVar = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> yVar = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> zVar = CreateObject<UniformRandomVariable>();
    xVar->SetAttribute("Min", DoubleValue(minX));
    xVar->SetAttribute("Max", DoubleValue(maxX));
    yVar->SetAttribute("Min", DoubleValue(minY));
    yVar->SetAttribute("Max", DoubleValue(maxY));
    zVar->SetAttribute("Min", DoubleValue(minHeight));
    zVar->SetAttribute("Max", DoubleValue(maxHeight));

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        posAlloc->Add(Vector(xVar->GetValue(), yVar->GetValue(), zVar->GetValue()));
    }

    MobilityHelper mobility;
    mobility.SetPositionAllocator(posAlloc);
    mobility.SetMobilityModel("ns3::GaussMarkovMobilityModel",
                              "Bounds",
                              BoxValue(Box(minX, maxX, minY, maxY, minHeight, maxHeight)),
                              "TimeStep",
                              TimeValue(Seconds(1.0)),
                              "Alpha",
                              DoubleValue(0.85),
                              "MeanVelocity",
                              StringValue("ns3::UniformRandomVariable[Min=0.3|Max=0.8]"),
                              "MeanDirection",
                              StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
                              "MeanPitch",
                              StringValue("ns3::UniformRandomVariable[Min=-0.05|Max=0.05]"),
                              "NormalVelocity",
                              StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
                              "NormalDirection",
                              StringValue(
                                  "ns3::NormalRandomVariable[Mean=0.0|Variance=0.1|Bound=0.2]"),
                              "NormalPitch",
                              StringValue(
                                  "ns3::NormalRandomVariable[Mean=0.0|Variance=0.01|Bound=0.02]"));
    mobility.Install(ueNodes);
}

void
ConfigureApMobility(NodeContainer& apNodes, double field, double apHeight)
{
    // 4-node square layout similar to wifi_mesh_simulation_file.cc
    std::vector<Vector> positions = {
        Vector(100.0, 100.0, apHeight),
        Vector(300.0, 100.0, apHeight),
        Vector(300.0, 300.0, apHeight),
        Vector(100.0, 300.0, apHeight),
    };

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < apNodes.GetN() && i < positions.size(); ++i)
    {
        posAlloc->Add(positions[i]);
    }

    MobilityHelper mob;
    mob.SetPositionAllocator(posAlloc);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(apNodes);
}

BuildingContainer
CreateBuildingObstacles(double field)
{
    BuildingContainer bc;

    // Basic 7-building layout (coarse approximation, consistent with previous files)
    Ptr<Building> b1 = CreateObject<Building>();
    b1->SetBoundaries(Box(50.0, 100.0, 150.0, 250.0, 0.0, 20.0));
    bc.Add(b1);

    Ptr<Building> b2 = CreateObject<Building>();
    b2->SetBoundaries(Box(150.0, 250.0, 50.0, 100.0, 0.0, 20.0));
    bc.Add(b2);

    Ptr<Building> b3 = CreateObject<Building>();
    b3->SetBoundaries(Box(275.0, 350.0, 150.0, 250.0, 0.0, 20.0));
    bc.Add(b3);

    Ptr<Building> b4 = CreateObject<Building>();
    b4->SetBoundaries(Box(150.0, 250.0, 275.0, 350.0, 0.0, 20.0));
    bc.Add(b4);

    Ptr<Building> b5 = CreateObject<Building>();
    b5->SetBoundaries(Box(175.0, 225.0, 175.0, 225.0, 0.0, 30.0));
    bc.Add(b5);

    Ptr<Building> b6 = CreateObject<Building>();
    b6->SetBoundaries(Box(0.0, 50.0, 0.0, 50.0, 0.0, 15.0));
    bc.Add(b6);

    Ptr<Building> b7 = CreateObject<Building>();
    b7->SetBoundaries(Box(field - 50.0, field, field - 50.0, field, 0.0, 15.0));
    bc.Add(b7);

    std::cout << "Total buildings: " << bc.GetN() << std::endl;
    return bc;
}

// ---------------------------------------------------------------------------//
// Cellular helpers (LTE and NR) - temporarily disabled
// ---------------------------------------------------------------------------//
//
// The following LTE/NR helper code is commented out for now while we bring
// up a pure WiFi-only hybrid scenario (STA -> AP -> remote host). Once the
// WiFi-only path is stable, we can progressively re-enable these helpers and
// add true WiFi+LTE/NR hybrid behaviour.
//
// struct CellularContext
// {
//     Ptr<Node> pgw;
//     Ptr<Node> remoteHost;
//     Ipv4InterfaceContainer ueIfaces;
//     Ipv4Address gateway;
// };
//
// CellularContext
// SetupLte(NodeContainer& bsNodes, NodeContainer& ueNodes, InternetStackHelper& internet)
// { ... }
//
// CellularContext
// SetupNr(NodeContainer& bsNodes, NodeContainer& ueNodes, InternetStackHelper& internet)
// { ... }

// ---------------------------------------------------------------------------//
// WiFi helpers
// ---------------------------------------------------------------------------//

struct WifiContext
{
    Ipv4InterfaceContainer apIfaces;
    Ipv4InterfaceContainer staIfaces;
    Ipv4Address gateway;
};

WifiContext
SetupWifiMeshLike(NodeContainer& apNodes,
                  NodeContainer& ueNodes,
                  InternetStackHelper& internet,
                  const MeshAPDeviceConfig& deviceCfg,
                  const std::string& hotspotBand)
{
    WifiContext ctx;

    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetErrorRateModel("ns3::NistErrorRateModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    WifiHelper wifi;

    // Apply mesh AP device preset
    wifi.SetStandard(deviceCfg.wifiStandard);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue(deviceCfg.dataMode),
                                 "ControlMode",
                                 StringValue(deviceCfg.dataMode));

    Ssid ssid("hybrid-wifi-mesh");

    // AP devices
    NetDeviceContainer apDevs;
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    apDevs = wifi.Install(wifiPhy, wifiMac, apNodes);

    // STA devices on UEs
    NetDeviceContainer staDevs;
    wifiMac.SetType("ns3::StaWifiMac",
                    "Ssid",
                    SsidValue(ssid),
                    "ActiveProbing",
                    BooleanValue(false));
    staDevs = wifi.Install(wifiPhy, wifiMac, ueNodes);

    // IP assignment
    Ipv4AddressHelper ip;
    ip.SetBase(kWifiSubnet, kWifiMask);
    ctx.apIfaces = ip.Assign(apDevs);
    // Advance base to avoid address collision between AP and STA ranges
    ip.NewNetwork();
    ctx.staIfaces = ip.Assign(staDevs);

    // Use AP 0 as WiFi gateway for now
    ctx.gateway = ctx.apIfaces.GetAddress(0);

    return ctx;
}

// ---------------------------------------------------------------------------//
// Traffic helpers (very simple placeholder)
// ---------------------------------------------------------------------------//

void
SetupHybridTraffic(const NodeContainer& ueNodes,
                   Ptr<Node> remoteHost,
                   Ipv4Address remoteHostIp,
                   uint32_t packetSize,
                   uint32_t httpBytes,
                   uint32_t httpsBytes,
                   uint32_t videoBytes,
                   uint32_t voipBytes,
                   uint32_t mixedBytes)
{
    uint16_t portBase = 9000;
    uint16_t port = portBase;

    // Remote host acts as server (sinks)
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Address sinkAddress(InetSocketAddress(remoteHostIp, port));
        PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", sinkAddress);
        ApplicationContainer sinkApp = sinkHelper.Install(remoteHost);
        sinkApp.Start(Seconds(0.5));
        sinkApp.Stop(Seconds(30.0));

        BulkSendHelper client("ns3::TcpSocketFactory", sinkAddress);
        client.SetAttribute("MaxBytes", UintegerValue(httpBytes + httpsBytes + videoBytes
                                                      + mixedBytes));
        ApplicationContainer clientApp = client.Install(ueNodes.Get(i));
        clientApp.Start(Seconds(1.0 + 0.1 * i));
        clientApp.Stop(Seconds(29.0));

        port++;
    }
}

// ---------------------------------------------------------------------------//
// Per-UE routing helpers (WiFi-only for now)
// ---------------------------------------------------------------------------//

void
ConfigureWifiOnlyRouting(const NodeContainer& ueNodes, const WifiContext& wifiCtx)
{
    Ipv4StaticRoutingHelper routingHelper;
    Ipv4Address defaultNetwork("0.0.0.0");
    Ipv4Mask defaultMask("0.0.0.0");

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ue = ueNodes.Get(i);
        Ptr<Ipv4> ipv4 = ue->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> rt = routingHelper.GetStaticRouting(ipv4);

        Ipv4Address wifiAddr = wifiCtx.staIfaces.GetAddress(i);

        int32_t wifiIfIndex = -1;
        uint32_t nIfaces = ipv4->GetNInterfaces();
        for (uint32_t ifIndex = 0; ifIndex < nIfaces; ++ifIndex)
        {
            uint32_t nAddrs = ipv4->GetNAddresses(ifIndex);
            for (uint32_t a = 0; a < nAddrs; ++a)
            {
                Ipv4InterfaceAddress ifAddr = ipv4->GetAddress(ifIndex, a);
                if (ifAddr.GetLocal() == wifiAddr)
                {
                    wifiIfIndex = static_cast<int32_t>(ifIndex);
                }
            }
        }

        if (wifiIfIndex < 0)
        {
            std::cerr << "[HYBRID] Warning: could not find WiFi interface for UE "
                      << i << " (wifiIfIndex=" << wifiIfIndex << "). Skipping routing config."
                      << std::endl;
            continue;
        }

        rt->AddNetworkRouteTo(defaultNetwork,
                              defaultMask,
                              wifiCtx.gateway,
                              static_cast<uint32_t>(wifiIfIndex),
                              /*metric*/ 1);

        std::cout << "[HYBRID] UE " << i << " WiFi-only routing: if=" << wifiIfIndex
                  << " gw=" << wifiCtx.gateway << " (metric 1)" << std::endl;
    }
}

// ---------------------------------------------------------------------------//
// Main
// ---------------------------------------------------------------------------//

int
main(int argc, char** argv)
{
    // CLI parameters
    std::string cellType = "lte"; // "lte" or "nr"
    uint32_t numUes = 10;
    double simTime = 30.0;
    uint32_t rngSeed = 6;
    uint32_t packetSize = 1400;
    uint32_t httpBytes = 1 * 1024 * 1024;
    uint32_t httpsBytes = 1 * 1024 * 1024;
    uint32_t videoBytes = 3 * 1024 * 1024;
    uint32_t voipBytes = 1 * 1024 * 1024;
    uint32_t mixedBytes = 1 * 1024 * 1024;
    std::string outputDir = "wifi_hybrid_outputs";
    uint32_t meshConfig = 1;           // same semantics as wifi_mesh_simulation_file.cc
    std::string hotspotBand = "5g";    // "5g" or "2g"

    CommandLine cmd(__FILE__);
    cmd.AddValue("cellType", "lte or nr", cellType);
    cmd.AddValue("numUes", "Number of hybrid UE nodes", numUes);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("rngSeed", "RNG seed", rngSeed);
    cmd.AddValue("packetSize", "Application packet size in bytes", packetSize);
    cmd.AddValue("httpBytes", "Total HTTP bytes per UE", httpBytes);
    cmd.AddValue("httpsBytes", "Total HTTPS bytes per UE", httpsBytes);
    cmd.AddValue("videoBytes", "Total Video bytes per UE", videoBytes);
    cmd.AddValue("voipBytes", "Total VoIP bytes per UE", voipBytes);
    cmd.AddValue("mixedBytes", "Total mixed bytes per UE", mixedBytes);
    cmd.AddValue("outputDir", "Output directory for traces/flowmon", outputDir);
    cmd.AddValue("meshConfig",
                 "Mesh AP device (0=Default 802.11g, 1=TP-Link EAP225, 2=Netgear Orbi 960, 3=Asus "
                 "ZenWiFi XT8)",
                 meshConfig);
    cmd.AddValue("hotspotBand", "Hotspot band for AP radios (5g or 2g)", hotspotBand);
    cmd.Parse(argc, argv);

    std::transform(cellType.begin(), cellType.end(), cellType.begin(), ::tolower);
    if (cellType != "lte" && cellType != "nr")
    {
        cellType = "lte";
    }

    RngSeedManager::SetSeed(rngSeed);
    PacketMetadata::Enable();
    Packet::EnablePrinting();
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(packetSize));
    fs::create_directories(fs::path(outputDir));

    // Normalize hotspot band
    std::string hotspotBandNorm = hotspotBand;
    std::transform(hotspotBandNorm.begin(),
                   hotspotBandNorm.end(),
                   hotspotBandNorm.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (hotspotBandNorm != "2g" && hotspotBandNorm != "5g")
    {
        NS_LOG_WARN("Invalid hotspotBand value '" << hotspotBand
                     << "'. Supported values are '5g' or '2g'. Defaulting to 5g.");
        hotspotBandNorm = "5g";
    }
    hotspotBand = hotspotBandNorm;

    if (meshConfig > 3)
    {
        NS_LOG_WARN("Invalid meshConfig value " << meshConfig << ", using default (0)");
        meshConfig = 0;
    }

    MeshAPDeviceConfig deviceCfg = GetMeshDeviceConfig(meshConfig);

    std::cout << "Hybrid scenario: WiFi + " << (cellType == "lte" ? "LTE" : "NR") << std::endl;
    std::cout << "  numUes=" << numUes << ", simTime=" << simTime << "s" << std::endl;

    // Nodes
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    NodeContainer apNodes;
    apNodes.Create(4);

    InternetStackHelper internet;
    internet.Install(ueNodes);
    internet.Install(apNodes);

    // Mobility and buildings
    ConfigureUeMobility(ueNodes, kFieldSize, kMinHeight, kMaxHeight);
    ConfigureApMobility(apNodes, kFieldSize, 10.0); // AP height ~10m
    BuildingContainer buildings = CreateBuildingObstacles(kFieldSize);
    BuildingsHelper::Install(ueNodes);
    BuildingsHelper::Install(apNodes);

    std::cout << "[HYBRID] Before SetupWifiMeshLike" << std::endl;
    // WiFi side (AP hardware preset + band selection).
    // NOTE: We currently rely on ns-3's default routing behaviour; explicit
    // WiFi-vs-cellular route metrics and switching logic will be added later.
    WifiContext wifiCtx = SetupWifiMeshLike(apNodes, ueNodes, internet, deviceCfg, hotspotBand);
    std::cout << "[HYBRID] After SetupWifiMeshLike" << std::endl;

    // Connect WiFi RSSI trace (MonitorSnifferRx) for future RSSI-based switching logic
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                    MakeCallback(&WifiSnifferRxCallback));

    // For now, create a simple remote host reachable via AP 0 using
    // a point-to-point link (WiFi-only scenario).
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    internet.Install(remoteHostContainer);

    // Position remote host outside the work area (for completeness)
    MobilityHelper remoteMob;
    Ptr<ListPositionAllocator> remotePos = CreateObject<ListPositionAllocator>();
    remotePos->Add(Vector(kFieldSize * 0.5, kFieldSize + 50.0, 0.0));
    remoteMob.SetPositionAllocator(remotePos);
    remoteMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    remoteMob.Install(remoteHostContainer);

    // Point-to-point backhaul between AP 0 and remote host
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));
    NetDeviceContainer p2pDevices = p2p.Install(apNodes.Get(0), remoteHost);

    Ipv4AddressHelper ipBackhaul;
    ipBackhaul.SetBase("1.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer backhaulIf = ipBackhaul.Assign(p2pDevices);
    Ipv4Address apBackhaulIp = backhaulIf.GetAddress(0);
    Ipv4Address remoteHostIp = backhaulIf.GetAddress(1);

    std::cout << "[HYBRID] remoteHost IP=" << remoteHostIp << std::endl;

    // Routing: currently rely on default behavior / EPC; explicit WiFi default
    // routes and WiFi↔cellular switching will be added later together with the
    // quality-based handover logic.

    // Per-UE routing: WiFi-only default routes via AP 0
    ConfigureWifiOnlyRouting(ueNodes, wifiCtx);

    // FlowMonitor for PDR / latency metrics
    std::cout << "[HYBRID] Before FlowMonitor install" << std::endl;
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> monitor = flowHelper.Install(ueNodes);
    flowHelper.Install(NodeContainer(remoteHost));
    std::cout << "[HYBRID] After FlowMonitor install" << std::endl;

    std::cout << "[HYBRID] Before SetupHybridTraffic" << std::endl;
    // Traffic: for now, send TCP bulk flows over WiFi to remoteHost
    SetupHybridTraffic(ueNodes,
                       remoteHost,
                       remoteHostIp,
                       packetSize,
                       httpBytes,
                       httpsBytes,
                       videoBytes,
                       voipBytes,
                       mixedBytes);
    std::cout << "[HYBRID] After SetupHybridTraffic" << std::endl;

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    std::string flowmonFile =
        outputDir + "/flowmon-wifi-hybrid-" + cellType + "-rw.xml";
    monitor->SerializeToXmlFile(flowmonFile, true, true);
    std::cout << "FlowMonitor XML saved to: " << flowmonFile << std::endl;

    Simulator::Destroy();
    return 0;
}

