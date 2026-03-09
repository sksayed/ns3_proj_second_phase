#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/trace-helper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/hybrid-buildings-propagation-loss-model.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/lte-enb-net-device.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/lte-ue-rrc.h"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cmath>
#include <vector>
#include <map>
#include <algorithm>
#include <array>
#include <filesystem>

using namespace ns3;
namespace fs = std::filesystem;

static const std::string kOutDir = "Lte_outputs";

// ============================================================================
// HELPER FUNCTION DECLARATIONS
// ============================================================================
void ConfigureUeMobility(NodeContainer& ueNodes,
                         double field,
                         double minHeight,
                         double maxHeight);
void ConfigureEnbMobility(NodeContainer& enbNodes, double field);
BuildingContainer CreateBuildingObstacles(double field);
void SetupInternetApplications(NodeContainer ueNodes,
                               Ptr<Node> remoteHost,
                               Ipv4InterfaceContainer ueIpIfaces,
                               double simStop,
                               uint32_t packetSize,
                               uint32_t httpBytes,
                               uint32_t httpsBytes,
                               uint32_t videoBytes,
                               uint32_t voipBytes,
                               uint32_t mixedBytes,
                               uint32_t extraHttpBytes);
void AttachUesToEnbs(Ptr<LteHelper> lteHelper,
                     NodeContainer ueNodes,
                     NodeContainer enbNodes,
                     NetDeviceContainer ueDevs,
                     NetDeviceContainer enbDevs);

// RRC trace callbacks for better runtime visibility
static void NotifyConnectionEstablishedUe(std::string context, uint64_t imsi,
                                          uint16_t cellid, uint16_t rnti) {
  std::cout << Simulator::Now().GetSeconds() << " " << context
            << " UE IMSI " << imsi << ": connected to CellId " << cellid
            << " with RNTI " << rnti << std::endl;
}

static void NotifyHandoverStartUe(std::string context, uint64_t imsi,
                                  uint16_t cellid, uint16_t rnti,
                                  uint16_t targetCellId) {
  std::cout << Simulator::Now().GetSeconds() << " " << context
            << " UE IMSI " << imsi << ": previously connected to CellId "
            << cellid << " with RNTI " << rnti << ", doing handover to CellId "
            << targetCellId << std::endl;
}

static void NotifyHandoverEndOkUe(std::string context, uint64_t imsi,
                                  uint16_t cellid, uint16_t rnti) {
  std::cout << Simulator::Now().GetSeconds() << " " << context
            << " UE IMSI " << imsi << ": successful handover to CellId "
            << cellid << " with RNTI " << rnti << std::endl;
}

static void NotifyConnectionEstablishedEnb(std::string context, uint64_t imsi,
                                           uint16_t cellid, uint16_t rnti) {
  std::cout << Simulator::Now().GetSeconds() << " " << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti << std::endl;
}

static void NotifyHandoverStartEnb(std::string context, uint64_t imsi,
                                   uint16_t cellid, uint16_t rnti,
                                   uint16_t targetCellId) {
  std::cout << Simulator::Now().GetSeconds() << " " << context
            << " eNB CellId " << cellid << ": start handover of UE with IMSI "
            << imsi << " RNTI " << rnti << " to CellId " << targetCellId
            << std::endl;
}

static void NotifyHandoverEndOkEnb(std::string context, uint64_t imsi,
                                   uint16_t cellid, uint16_t rnti) {
  std::cout << Simulator::Now().GetSeconds() << " " << context
            << " eNB CellId " << cellid << ": completed handover of UE with IMSI "
            << imsi << " RNTI " << rnti << std::endl;
}

// Output file name constants for easy configuration
static const std::string kPcapPrefix = "lte_playfield_rw_pcap";
static const std::string kAsciiTracesPrefix = "lte_playfield_ascii_traces";
static const std::string kNetAnimFile = "netanim-lte-playfield-rw.xml";
static const std::string kFlowmonFile = "flowmon-lte-playfield-rw.xml";

int main(int argc, char **argv) {
  // Basics
  PacketMetadata::Enable();
  Packet::EnablePrinting();
  

  uint32_t nUes = 10;
  const double field = 400.0;
  const double simStop = 30.0;
  const double minHeight = 0.0;
  const double maxHeight = 30.0;
  uint32_t rngSeed = 1;
  uint32_t packetSize = 1400;
  uint32_t httpBytes = 1 * 1024 * 1024;
  uint32_t httpsBytes = 1 * 1024 * 1024;
  uint32_t videoBytes = 3 * 1024 * 1024;
  uint32_t voipBytes = 1 * 1024 * 1024;
  uint32_t mixedBytes = 1 * 1024 * 1024;
  uint32_t extraHttpBytes = 1 * 1024 * 1024;
  std::string outputDir = kOutDir;
  double flowScale = 1.0;

  CommandLine cmd(__FILE__);
  cmd.AddValue("nUes", "Number of UE nodes to create", nUes);
  cmd.AddValue("rngSeed", "RNG seed for reproducible runs", rngSeed);
  cmd.AddValue("packetSize", "Application packet size in bytes for TCP/UDP flows", packetSize);
  cmd.AddValue("httpBytes", "Total bytes for each HTTP BulkSend flow", httpBytes);
  cmd.AddValue("httpsBytes", "Total bytes for each HTTPS BulkSend flow", httpsBytes);
  cmd.AddValue("videoBytes", "Total bytes for each video BulkSend flow", videoBytes);
  cmd.AddValue("voipBytes", "Total bytes for each VoIP OnOff flow", voipBytes);
  cmd.AddValue("mixedBytes", "Total bytes for each mixed BulkSend flow", mixedBytes);
  cmd.AddValue("extraHttpBytes", "Total bytes for the additional HTTP BulkSend flow", extraHttpBytes);
  cmd.AddValue("outputDir", "Directory where run artifacts (XML/metrics) are stored", outputDir);
  cmd.AddValue("flowScale",
               "Multiplier applied to HTTP/HTTPS/Video/VoIP/Mixed/Extra HTTP byte budgets (1=default)",
               flowScale);
  cmd.Parse(argc, argv);

  RngSeedManager::SetSeed(rngSeed);
  Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(packetSize));
  fs::create_directories(fs::path(outputDir));

  auto scaleBytes = [flowScale](uint32_t value) -> uint32_t {
    return static_cast<uint32_t>(std::max(1.0, std::round(static_cast<double>(value) * flowScale)));
  };
  httpBytes = scaleBytes(httpBytes);
  httpsBytes = scaleBytes(httpsBytes);
  videoBytes = scaleBytes(videoBytes);
  voipBytes = scaleBytes(voipBytes);
  mixedBytes = scaleBytes(mixedBytes);
  extraHttpBytes = scaleBytes(extraHttpBytes);

  
  NodeContainer ueNodes;
  ueNodes.Create(nUes);
  NodeContainer enbNodes;
  enbNodes.Create(1);

  ConfigureUeMobility(ueNodes, field, minHeight, maxHeight);
  ConfigureEnbMobility(enbNodes, field);

  BuildingContainer buildings = CreateBuildingObstacles(field);
  BuildingsHelper::Install(ueNodes);
  BuildingsHelper::Install(enbNodes);

  // Macro-cell transmit power similar to NR configuration
  Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(43.0)); // dBm
  Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(15.0));  // dBm
  std::cout << "TxPower settings: eNB=43.00 dBm, UE=15.00 dBm" << std::endl;
  std::cout << "UE initial position mode: uniform random distribution" << std::endl;

  // LTE + EPC
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
  Ptr<PointToPointEpcHelper> epcHelperP2p =
      CreateObject<PointToPointEpcHelper>();
  lteHelper->SetEpcHelper(epcHelperP2p);
  lteHelper->SetPathlossModelType(HybridBuildingsPropagationLossModel::GetTypeId());
  lteHelper->SetPathlossModelAttribute("Frequency", DoubleValue(2.0e9));
  lteHelper->SetUeAntennaModelType("ns3::IsotropicAntennaModel");
  lteHelper->SetUeAntennaModelAttribute("Gain", DoubleValue(1.0));
  Ptr<EpcHelper> epcHelper = epcHelperP2p;

  NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);
  NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ueNodes);

  // Enable X2 and LTE traces for better visualization
  lteHelper->EnablePhyTraces();
  lteHelper->EnableMacTraces();
  lteHelper->EnableRlcTraces();
  lteHelper->EnablePdcpTraces();

  // Connect RRC trace sinks
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                  MakeCallback(&NotifyConnectionEstablishedEnb));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                  MakeCallback(&NotifyConnectionEstablishedUe));
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                  MakeCallback(&NotifyHandoverStartEnb));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                  MakeCallback(&NotifyHandoverStartUe));
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                  MakeCallback(&NotifyHandoverEndOkEnb));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                  MakeCallback(&NotifyHandoverEndOkUe));

  // Internet stack on UEs via EPC-assigned IPs
  InternetStackHelper internet;
  internet.Install(ueNodes);
  Ipv4InterfaceContainer ueIpIfaces =
      epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(i)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
  }

  AttachUesToEnbs(lteHelper, ueNodes, enbNodes, ueDevs, enbDevs);

  // Report eNB Macro layout
  {
    std::cout << "eNB positions:" << std::endl;
    std::vector<Vector> enbPositions;
    enbPositions.reserve(enbNodes.GetN());
    for (uint32_t e = 0; e < enbNodes.GetN(); ++e) {
      Ptr<MobilityModel> mm = enbNodes.Get(e)->GetObject<MobilityModel>();
      Vector p = mm->GetPosition();
      enbPositions.push_back(p);
      std::cout << "  eNB" << e << ": (" << p.x << ", " << p.y << ", " << p.z
                << ")" << std::endl;
    }
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "eNB pairwise distances (m):" << std::endl;
    for (uint32_t i = 0; i < enbPositions.size(); ++i) {
      for (uint32_t j = i + 1; j < enbPositions.size(); ++j) {
        double dx = enbPositions[i].x - enbPositions[j].x;
        double dy = enbPositions[i].y - enbPositions[j].y;
        double dz = enbPositions[i].z - enbPositions[j].z;
        double d = std::sqrt(dx * dx + dy * dy + dz * dz);
        std::cout << "  eNB" << i << "-eNB" << j << ": " << d << std::endl;
      }
    }
  }



  // Create Remote Host to hook PGW and generate pcap/ascii on core link
  Ptr<Node> pgw = epcHelper->GetPgwNode();
  Ptr<Node> remoteHost = CreateObject<Node>();
  NodeContainer remoteHostContainer(remoteHost);
  internet.Install(remoteHostContainer);

  // Add mobility model to remoteHost to avoid AnimationInterface warnings
  MobilityHelper remoteHostMob;
  Ptr<ListPositionAllocator> remoteHostPos =
      CreateObject<ListPositionAllocator>();
  remoteHostPos->Add(Vector(field * 0.5, field + 50.0,
                            0.0)); // Position remote host outside the field
  remoteHostMob.SetPositionAllocator(remoteHostPos);
  remoteHostMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  remoteHostMob.Install(remoteHostContainer);

  // Add mobility models to EPC nodes to avoid AnimationInterface warnings
  Ptr<Node> sgw = epcHelper->GetSgwNode();
  NodeContainer epcNodes;
  epcNodes.Add(pgw);
  epcNodes.Add(sgw);
  MobilityHelper epcMob;
  Ptr<ListPositionAllocator> epcPos = CreateObject<ListPositionAllocator>();
  epcPos->Add(
      Vector(field * 0.5, field + 100.0, 0.0)); // Position PGW near remote host
  epcPos->Add(Vector(field * 0.3, field + 100.0, 0.0)); // Position SGW
  epcMob.SetPositionAllocator(epcPos);
  epcMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  epcMob.Install(epcNodes);

  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute("DataRate", StringValue("100Gbps"));
  p2ph.SetChannelAttribute("Delay", StringValue("1ms"));
  NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  ipv4h.Assign(internetDevices); // Assign IP addresses to internet devices

  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
  remoteHostStaticRouting->SetDefaultRoute(
      epcHelper->GetUeDefaultGatewayAddress(), 1);

  // // Enable pcap and ascii traces on EPC P2P link with requested prefix
  // p2ph.EnablePcapAll(kOutDir + "/" + kPcapPrefix, true);
  // AsciiTraceHelper ascii;
  // Ptr<OutputStreamWrapper> stream =
  //     ascii.CreateFileStream(kOutDir + "/" + kAsciiTracesPrefix + ".tr");
  // p2ph.EnableAsciiAll(stream);

  SetupInternetApplications(ueNodes,
                            remoteHost,
                            ueIpIfaces,
                            simStop,
                            packetSize,
                            httpBytes,
                            httpsBytes,
                            videoBytes,
                            voipBytes,
                            mixedBytes,
                            extraHttpBytes);

  // FlowMonitor
  FlowMonitorHelper fm;
  Ptr<FlowMonitor> monitor = fm.InstallAll();

  Simulator::Stop(Seconds(simStop));
  Simulator::Run();

  monitor->SerializeToXmlFile(outputDir + "/" + kFlowmonFile, true, true);
  std::cout << "FlowMonitor XML saved to: " << outputDir << "/" << kFlowmonFile << std::endl;
  std::cout << "All results saved to: " << outputDir << "/" << std::endl;

  std::ostringstream parseCmd;
  parseCmd << "python3 examples/my-scenarios/parse_lte_flowmon.py"
           << " --xml " << outputDir << "/" << kFlowmonFile
           << " --sim-time=" << simStop
           << " --md " << outputDir << "/lte-playfield-metrics.md";

  int parseStatus = std::system(parseCmd.str().c_str());
  if (parseStatus != 0) {
    std::cerr << "FlowMonitor parser exited with status " << parseStatus << std::endl;
  }

  Simulator::Destroy();
  return 0;
}


void ConfigureUeMobility(NodeContainer& ueNodes,
                         double field,
                         double minHeight,
                         double maxHeight) {
  std::cout << "\n=== Configuring UE Mobility (Gauss-Markov) ===" << std::endl;

  double minX = 0.0;
  double maxX = field;
  double minY = 0.0;
  double maxY = field;
  double minZ = minHeight;
  double maxZ = maxHeight;

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();

  Ptr<UniformRandomVariable> xVar = CreateObject<UniformRandomVariable>();
  Ptr<UniformRandomVariable> yVar = CreateObject<UniformRandomVariable>();
  Ptr<UniformRandomVariable> zVar = CreateObject<UniformRandomVariable>();
  xVar->SetAttribute("Min", DoubleValue(minX));
  xVar->SetAttribute("Max", DoubleValue(maxX));
  yVar->SetAttribute("Min", DoubleValue(minY));
  yVar->SetAttribute("Max", DoubleValue(maxY));
  zVar->SetAttribute("Min", DoubleValue(minZ));
  zVar->SetAttribute("Max", DoubleValue(maxZ));

  for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
  {
    posAlloc->Add(Vector(xVar->GetValue(), yVar->GetValue(), zVar->GetValue()));
  }

  std::cout << "Initial positions: uniform random distribution across field bounds" << std::endl;

  mobility.SetPositionAllocator(posAlloc);
  mobility.SetMobilityModel("ns3::GaussMarkovMobilityModel",
                            "Bounds", BoxValue(Box(minX, maxX, minY, maxY, minZ, maxZ)),
                            "TimeStep", TimeValue(Seconds(1.0)),
                            "Alpha", DoubleValue(0.85),
                            "MeanVelocity", StringValue("ns3::UniformRandomVariable[Min=0.3|Max=0.8]"),
                            "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
                            "MeanPitch", StringValue("ns3::UniformRandomVariable[Min=-0.05|Max=0.05]"),
                            "NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
                            "NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.1|Bound=0.2]"),
                            "NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.01|Bound=0.02]"));

  mobility.Install(ueNodes);

  std::cout << "Movement Type: Smooth pedestrian (Gauss-Markov)" << std::endl;
  std::cout << "Bounds: (" << minX << "," << maxX << ") × (" << minY << "," << maxY
            << ") × (" << minZ << "," << maxZ << ")" << std::endl;
  std::cout << "====================================\n" << std::endl;
}

void ConfigureEnbMobility(NodeContainer& enbNodes, double field) {
  std::cout << "=== Configuring eNB Macro Positions ===" << std::endl;
  MobilityHelper enbMob;
  Ptr<ListPositionAllocator> enbPos = CreateObject<ListPositionAllocator>();
  enbPos->Add(Vector(200.0, 200.0, 30.0));
  enbMob.SetPositionAllocator(enbPos);
  enbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  enbMob.Install(enbNodes);

  for (uint32_t e = 0; e < enbNodes.GetN(); ++e)
  {
    Ptr<MobilityModel> mm = enbNodes.Get(e)->GetObject<MobilityModel>();
    Vector p = mm->GetPosition();
    std::cout << "  eNB" << e << ": (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
  }
  std::cout << "====================================\n" << std::endl;
}

BuildingContainer CreateBuildingObstacles(double field) {
  std::cout << "=== Creating Static Building Obstacles ===" << std::endl;
  std::cout << "Field size reference: " << field << " m × " << field << " m" << std::endl;
  BuildingContainer buildings;

  Ptr<Building> leftBelow = CreateObject<Building>();
  leftBelow->SetBoundaries(Box(0.0, 60.0, 96.0, 104.0, 0.0, 10.0));
  buildings.Add(leftBelow);

  Ptr<Building> rightBelow = CreateObject<Building>();
  rightBelow->SetBoundaries(Box(340.0, 400.0, 96.0, 104.0, 0.0, 10.0));
  buildings.Add(rightBelow);

  Ptr<Building> leftAbove = CreateObject<Building>();
  leftAbove->SetBoundaries(Box(0.0, 60.0, 296.0, 304.0, 0.0, 10.0));
  buildings.Add(leftAbove);

  Ptr<Building> rightAbove = CreateObject<Building>();
  rightAbove->SetBoundaries(Box(340.0, 400.0, 296.0, 304.0, 0.0, 10.0));
  buildings.Add(rightAbove);

  Ptr<Building> cluster250a = CreateObject<Building>();
  cluster250a->SetBoundaries(Box(80.0, 140.0, 300.0, 308.0, 0.0, 15.0));
  buildings.Add(cluster250a);

  Ptr<Building> cluster250b = CreateObject<Building>();
  cluster250b->SetBoundaries(Box(170.0, 250.0, 300.0, 308.0, 0.0, 12.0));
  buildings.Add(cluster250b);

  Ptr<Building> cluster50 = CreateObject<Building>();
  cluster50->SetBoundaries(Box(255.0, 335.0, 20.0, 28.0, 0.0, 18.0));
  buildings.Add(cluster50);

  std::cout << "Total buildings: " << buildings.GetN() << std::endl;
  std::cout << "Static obstacles configured (no movement events)" << std::endl;
  std::cout << "====================================\n" << std::endl;

  return buildings;
}

void AttachUesToEnbs(Ptr<LteHelper> lteHelper,
                     NodeContainer ueNodes,
                     NodeContainer enbNodes,
                     NetDeviceContainer ueDevs,
                     NetDeviceContainer enbDevs)
{
  std::cout << "=== Attaching UEs via LTE measurement framework ===" << std::endl;

  if (enbNodes.GetN() > 1)
  {
    lteHelper->AddX2Interface(enbNodes);
  }

  lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
  lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
  lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(160)));

  lteHelper->AttachToClosestEnb(ueDevs, enbDevs);

  std::map<uint16_t, std::pair<uint32_t, Vector>> cellIdToInfo;
  for (uint32_t e = 0; e < enbNodes.GetN(); ++e)
  {
    Ptr<LteEnbNetDevice> enbDev = DynamicCast<LteEnbNetDevice>(enbDevs.Get(e));
    NS_ABORT_MSG_IF(enbDev == nullptr, "AttachUesToEnbs: expected LteEnbNetDevice");
    uint16_t cellId = enbDev->GetCellId();
    Vector pos = enbNodes.Get(e)->GetObject<MobilityModel>()->GetPosition();
    cellIdToInfo.emplace(cellId, std::make_pair(e, pos));
  }

  for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
  {
    Ptr<MobilityModel> ueMob = ueNodes.Get(i)->GetObject<MobilityModel>();
    NS_ABORT_MSG_IF(ueMob == nullptr, "AttachUesToEnbs: UE " << i << " missing MobilityModel");
    Vector uePos = ueMob->GetPosition();

    Ptr<LteUeNetDevice> ueDev = DynamicCast<LteUeNetDevice>(ueDevs.Get(i));
    NS_ABORT_MSG_IF(ueDev == nullptr, "AttachUesToEnbs: expected LteUeNetDevice");
    Ptr<LteUeRrc> ueRrc = ueDev->GetRrc();
    uint16_t servingCellId = ueRrc->GetCellId();

    uint32_t servingIndex = 0;
    Vector enbPos;
    auto cellIt = cellIdToInfo.find(servingCellId);
    if (cellIt != cellIdToInfo.end())
    {
      servingIndex = cellIt->second.first;
      enbPos = cellIt->second.second;
    }
    else
    {
      enbPos = enbNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
    }

    double dx = uePos.x - enbPos.x;
    double dy = uePos.y - enbPos.y;
    double dz = uePos.z - enbPos.z;
    double groundDist = std::sqrt(dx * dx + dy * dy);
    double spatialDist = std::sqrt(dx * dx + dy * dy + dz * dz);

    std::cout << "UE " << i << " attached to eNB" << servingIndex << " (CellId=" << servingCellId
              << ") | UE=(" << uePos.x << ", " << uePos.y << ", " << uePos.z << ") eNB=("
              << enbPos.x << ", " << enbPos.y << ", " << enbPos.z
              << ") groundDist=" << std::fixed << std::setprecision(2) << groundDist
              << "m spatialDist=" << spatialDist << "m" << std::endl;
  }

}

void SetupInternetApplications(NodeContainer ueNodes,
                               Ptr<Node> remoteHost,
                               Ipv4InterfaceContainer ueIpIfaces,
                               double simStop,
                               uint32_t packetSize,
                               uint32_t httpBytes,
                               uint32_t httpsBytes,
                               uint32_t videoBytes,
                               uint32_t voipBytes,
                               uint32_t mixedBytes,
                               uint32_t extraHttpBytes) {
  std::cout << "\n=== Setting Up Internet Applications ===" << std::endl;

  Ipv4Address remoteHostAddr =
      remoteHost->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
  std::cout << "Remote Host (Internet Server) IP: " << remoteHostAddr << std::endl;

  const uint16_t httpPort = 80;
  const uint16_t httpsPort = 443;
  const uint16_t videoPort = 8080;
  const uint16_t voipPort = 5060;

  ApplicationContainer serverApps;
  PacketSinkHelper httpServer("ns3::TcpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), httpPort));
  serverApps.Add(httpServer.Install(remoteHost));

  PacketSinkHelper httpsServer("ns3::TcpSocketFactory",
                               InetSocketAddress(Ipv4Address::GetAny(), httpsPort));
  serverApps.Add(httpsServer.Install(remoteHost));

  PacketSinkHelper videoServer("ns3::TcpSocketFactory",
                               InetSocketAddress(Ipv4Address::GetAny(), videoPort));
  serverApps.Add(videoServer.Install(remoteHost));

  UdpServerHelper voipServer(voipPort);
  serverApps.Add(voipServer.Install(remoteHost));

  serverApps.Start(Seconds(0.5));
  serverApps.Stop(Seconds(simStop));

  enum class FlowType { Http, Https, Video, Voip, Mixed };

  struct FlowPattern
  {
    FlowType type;
    uint32_t maxBytes;
    double startTime;
  };

  const std::vector<FlowPattern> kFlowPattern = {
      {FlowType::Http, httpBytes, 10.0},   {FlowType::Http, httpBytes, 10.1},
      {FlowType::Https, httpsBytes, 10.2}, {FlowType::Https, httpsBytes, 10.3},
      {FlowType::Video, videoBytes, 10.4}, {FlowType::Video, videoBytes, 10.5},
      {FlowType::Voip, voipBytes, 10.6},   {FlowType::Voip, voipBytes, 10.7},
      {FlowType::Http, extraHttpBytes, 10.8},
      {FlowType::Mixed, mixedBytes, 11.0},
  };

  const InetSocketAddress httpAddress(remoteHostAddr, httpPort);
  const InetSocketAddress httpsAddress(remoteHostAddr, httpsPort);
  const InetSocketAddress videoAddress(remoteHostAddr, videoPort);
  const InetSocketAddress voipAddress(remoteHostAddr, voipPort);

  ApplicationContainer clientApps;

  for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
    const uint32_t patternIndex =
        (i + kFlowPattern.size() / 2) % kFlowPattern.size();
    const FlowPattern &pattern = kFlowPattern[patternIndex];
    double cycleOffset = static_cast<double>(i / kFlowPattern.size());
    double startTime = pattern.startTime + cycleOffset;

    switch (pattern.type) {
    case FlowType::Http:
    case FlowType::Mixed: {
      BulkSendHelper sender("ns3::TcpSocketFactory", videoAddress);
      sender.SetAttribute("MaxBytes", UintegerValue(pattern.maxBytes));
      sender.SetAttribute("SendSize", UintegerValue(packetSize));
      ApplicationContainer app = sender.Install(ueNodes.Get(i));
      app.Start(Seconds(startTime));
      app.Stop(Seconds(simStop));
      clientApps.Add(app);
      break;
    }
    case FlowType::Https: {
      BulkSendHelper sender("ns3::TcpSocketFactory", videoAddress);
      sender.SetAttribute("MaxBytes", UintegerValue(pattern.maxBytes));
      sender.SetAttribute("SendSize", UintegerValue(packetSize));
      ApplicationContainer app = sender.Install(ueNodes.Get(i));
      app.Start(Seconds(startTime));
      app.Stop(Seconds(simStop));
      clientApps.Add(app);
      break;
    }
    case FlowType::Video: {
      BulkSendHelper sender("ns3::TcpSocketFactory", videoAddress);
      sender.SetAttribute("MaxBytes", UintegerValue(pattern.maxBytes));
      sender.SetAttribute("SendSize", UintegerValue(packetSize));
      ApplicationContainer app = sender.Install(ueNodes.Get(i));
      app.Start(Seconds(startTime));
      app.Stop(Seconds(simStop));
      clientApps.Add(app);
      break;
    }
    case FlowType::Voip: {
      OnOffHelper client("ns3::UdpSocketFactory", voipAddress);
      client.SetAttribute("DataRate", DataRateValue(DataRate("1.5Mbps")));
      client.SetAttribute("PacketSize", UintegerValue(packetSize));
      client.SetAttribute(
          "OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
      client.SetAttribute(
          "OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
      client.SetAttribute("MaxBytes", UintegerValue(pattern.maxBytes));
      ApplicationContainer app = client.Install(ueNodes.Get(i));
      app.Start(Seconds(startTime));
      app.Stop(Seconds(simStop));
      clientApps.Add(app);
      break;
    }
    }
  }
}