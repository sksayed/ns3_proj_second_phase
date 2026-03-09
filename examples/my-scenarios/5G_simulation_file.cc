/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2024
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: 5G NR Playfield Simulation with Traces
 */

#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ideal-beamforming-algorithm.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/pointer.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/hybrid-buildings-propagation-loss-model.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <array>
#include <filesystem>
#include <algorithm>

using namespace ns3;
namespace fs = std::filesystem;

static const std::string kOutDir = "5G_outputs";

// ============================================================================
// HELPER FUNCTION DECLARATIONS
// ============================================================================

// Mobility configuration
void ConfigureUeMobility(NodeContainer& ueNodes,
                         double field,
                         double minHeight,
                         double maxHeight);
void ConfigureGnbMobility(NodeContainer& gnbNodes, double field);

// Network infrastructure
BuildingContainer CreateBuildingObstacles(double field);

// Node setup
void AttachUesToGnbs(Ptr<NrHelper> nrHelper,
                     NodeContainer ueNodes,
                     NodeContainer gnbNodes,
                     NetDeviceContainer ueDevs,
                     NetDeviceContainer gnbDevs);

// Application configuration
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

// ============================================================================
// RRC trace callbacks for better runtime visibility
// ============================================================================
static void
NotifyConnectionEstablishedUe(std::string context,
                              uint64_t imsi,
                              uint16_t cellid,
                              uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi
              << ": connected to CellId " << cellid << " with RNTI " << rnti << std::endl;
}

static void
NotifyHandoverStartUe(std::string context,
                      uint64_t imsi,
                      uint16_t cellid,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi
              << ": previously connected to CellId " << cellid << " with RNTI " << rnti
              << ", doing handover to CellId " << targetCellId << std::endl;
}

static void
NotifyHandoverEndOkUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi
              << ": successful handover to CellId " << cellid << " with RNTI " << rnti << std::endl;
}

static void
NotifyConnectionEstablishedEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " gNB CellId " << cellid
              << ": successful connection of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
}

static void
NotifyHandoverStartEnb(std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " gNB CellId " << cellid
              << ": start handover of UE with IMSI " << imsi << " RNTI " << rnti << " to CellId "
              << targetCellId << std::endl;
}

static void
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " gNB CellId " << cellid
              << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
}

// Function to update building position dynamically
void
UpdateBuildingPosition(Ptr<Building> building, Vector newPosition, double width, double height)
{
    Box newBounds(newPosition.x,
                  newPosition.x + width,
                  newPosition.y,
                  newPosition.y + height,
                  0.0,
                  10.0);
    building->SetBoundaries(newBounds);
    std::cout << "Building moved to (" << newPosition.x << ", " << newPosition.y << ")"
              << std::endl;
}

// Output file name constants for easy configuration
static const std::string kPcapPrefix = "nr_playfield_rw_pcap";
static const std::string kAsciiTracesPrefix = "nr_playfield_ascii_traces";
static const std::string kNetAnimFile = "netanim-nr-playfield-rw.xml";
static const std::string kFlowmonFile = "flowmon-nr-playfield-rw.xml";

int
main(int argc, char** argv)
{
    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    uint32_t nUes = 10;
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

    PacketMetadata::Enable();
    Packet::EnablePrinting();
    


    // ========================================================================
    // SIMULATION PARAMETERS
    // ========================================================================
    const double field = 400.0;
    const double simStop = 30.0;  
    const double minHeight = 0.0;
    const double maxHeight = 30.0;

    std::cout << "Simulation Parameters:" << std::endl;
    std::cout << "  Number of UEs: " << nUes << std::endl;
    std::cout << "  Stage size (L×B×H): " << field << "m × " << field << "m × " << maxHeight << "m" << std::endl;
    std::cout << "  Ground elevation range: " << minHeight << "m - " << maxHeight << "m" << std::endl;
    std::cout << "  Simulation time: " << simStop << " seconds\n" << std::endl;
    std::cout << "  UE initial position mode: uniform random distribution" << std::endl;

    
    NodeContainer ueNodes;
    ueNodes.Create(nUes);
    NodeContainer gnbNodes;
    gnbNodes.Create(1);

 
    ConfigureUeMobility(ueNodes, field, minHeight, maxHeight);
    ConfigureGnbMobility(gnbNodes, field);

   
    BuildingContainer buildings = CreateBuildingObstacles(field);

  
    
    // Create NR helper
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    // Align UE transmit characteristics with Wi-Fi STA profile
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(15.0));// this has been set up by finding minimalistic value for UE transmit power
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    Ptr<IsotropicAntennaModel> ueAntenna = CreateObject<IsotropicAntennaModel>();
    ueAntenna->SetAttribute("Gain", DoubleValue(1.0));
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(ueAntenna));

    
    // Create EPC helper
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    
    // Create beamforming helper
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    
    // Configure spectrum
    double centralFrequency = 3.5e9;
    double bandwidth = 100e6;
    
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, numCcPerBand);
    bandConf.m_numBwp = 1;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    
    // Create channel helper with shared HybridBuildings propagation model
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("RMa", "Default", "ThreeGpp");
    channelHelper->ConfigurePropagationFactory(
        HybridBuildingsPropagationLossModel::GetTypeId());
    channelHelper->SetPathlossAttribute("Frequency", DoubleValue(centralFrequency));
    std::cout << "Channel Model: HybridBuildingsPropagationLossModel" << std::endl;
    
    // Set beamforming method
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                        TypeIdValue(DirectPathBeamforming::GetTypeId()));
    
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});
    
    // Configure BWP manager
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));
    
  
    std::cout << "=== Installing NR Devices ===" << std::endl;
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    
    BuildingsHelper::Install(ueNodes);
    BuildingsHelper::Install(gnbNodes);
    // nrHelper->EnableTraces();


  
    
    // Configure TCP with larger buffer sizes for 1 MB packets
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(10 * 1024 * 1024)); // 10 MB
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(10 * 1024 * 1024)); // 10 MB
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(packetSize)); // Align with flow packet size
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));
    
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
   

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        NS_ABORT_MSG_IF(ueNodes.Get(i)->GetObject<MobilityModel>() == nullptr,
                        "UE node " << i << " is missing a MobilityModel");
    }
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        NS_ABORT_MSG_IF(gnbNodes.Get(i)->GetObject<MobilityModel>() == nullptr,
                        "gNB node " << i << " is missing a MobilityModel");
    }

    AttachUesToGnbs(nrHelper, ueNodes, gnbNodes, ueDevs, gnbDevs);

  

    // Create Remote Host to hook PGW and generate pcap/ascii on core link
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<Node> remoteHost = CreateObject<Node>();
    NodeContainer remoteHostContainer(remoteHost);
    internet.Install(remoteHostContainer);

    // Add mobility model to remoteHost to avoid AnimationInterface warnings
    MobilityHelper remoteHostMob;
    Ptr<ListPositionAllocator> remoteHostPos = CreateObject<ListPositionAllocator>();
    remoteHostPos->Add(Vector(field * 0.5, field + 50.0, 0.0)); // Position remote host outside
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
    epcPos->Add(Vector(field * 0.5, field + 100.0, 0.0)); // Position PGW near remote host
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

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    
 
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

  
    
    // FlowMonitor for network statistics
    std::cout << "=== Setting Up Monitoring ===" << std::endl;
    FlowMonitorHelper fm;
    Ptr<FlowMonitor> monitor = fm.InstallAll();
    std::cout << "FlowMonitor installed" << std::endl;

    // // NetAnim for visualization (disabled)
    // anim.UpdateNodeDescription(sgw, "SGW");
    // anim.UpdateNodeColor(sgw, 255, 0, 255);
    
    // std::cout << "NetAnim configured successfully" << std::endl;

    // // IPv4 L3 ASCII tracing
    // AsciiTraceHelper asciiL3;
    // Ptr<OutputStreamWrapper> ipStream = asciiL3.CreateFileStream(kOutDir + "/ipv4-l3.tr");
    // internet.EnableAsciiIpv4All(ipStream);
    // std::cout << "IPv4 L3 traces enabled" << std::endl;

    std::cout << "\n*** Starting simulation for " << simStop << " seconds ***\n" << std::endl;
    
    Simulator::Stop(Seconds(simStop));
    Simulator::Run();
    
  
    
    // Save FlowMonitor results
    monitor->SerializeToXmlFile(outputDir + "/" + kFlowmonFile, true, true);
    
    std::cout << "FlowMonitor XML saved to: " << outputDir << "/" << kFlowmonFile << std::endl;
    std::cout << "All results saved to: " << outputDir << "/" << std::endl;
    
    std::ostringstream parseCmd;
    parseCmd << "cd /home/sayed/ns3_phase_2/ns-3.45 && "
             << "python3 examples/my-scenarios/parse_nr_flowmon.py"
             << " --xml " << outputDir << "/" << kFlowmonFile
             << " --sim-time=" << simStop
             << " --md " << outputDir << "/nr-playfield-metrics.md";

  
    int parseStatus = std::system(parseCmd.str().c_str());
    if (parseStatus != 0)
    {
        std::cerr << "FlowMonitor parser exited with status " << parseStatus << std::endl;
    }

Simulator::Destroy();
    return 0;
}

// ============================================================================
/**
 * Configure UE mobility using Gauss-Markov model
 * @param ueNodes Container of UE nodes
 * @param field Field size in meters
 * @param minHeight Minimum height for UE movement (meters)
 * @param maxHeight Maximum height for UE movement (meters)
 */
void
ConfigureUeMobility(NodeContainer& ueNodes,
                    double field,
                    double minHeight,
                    double maxHeight)
{
    std::cout << "\n=== Configuring UE Mobility ===" << std::endl;
    
    // Define movement bounds
    double minX = 0.0;
    double maxX = field;
    double minY = 0.0;
    double maxY = field;
    double minZ = minHeight;
    double maxZ = maxHeight;
    
    // Initial positions: distribute UEs across the field with varying heights
    MobilityHelper staMobility;
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

    staMobility.SetPositionAllocator(posAlloc);
    staMobility.SetMobilityModel("ns3::GaussMarkovMobilityModel",
        "Bounds", BoxValue(Box(minX, maxX, minY, maxY, minZ, maxZ)),
        "TimeStep", TimeValue(Seconds(1.0)),
        "Alpha", DoubleValue(0.85),
        "MeanVelocity", StringValue("ns3::UniformRandomVariable[Min=0.3|Max=0.8]"),
        "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
        "MeanPitch", StringValue("ns3::UniformRandomVariable[Min=-0.05|Max=0.05]"),
        "NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
        "NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.1|Bound=0.2]"),
        "NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.01|Bound=0.02]"));
    
    staMobility.Install(ueNodes);
    
    std::cout << "Movement Type: Smooth, correlated pedestrian movement" << std::endl;
    std::cout << "Speed Range: 0.3 - 0.8 m/s (slow walking)" << std::endl;
    std::cout << "Alpha: 0.85 (high memory, smooth trajectories)" << std::endl;
    std::cout << "Bounds: (" << minX << "," << maxX << ") × (" << minY << "," << maxY << ") × ("
              << minZ << "," << maxZ << ")" << std::endl;
    std::cout << "====================================\n" << std::endl;
}

/**
 * Configure gNB (base station) positions
 * @param gnbNodes Container of gNB nodes
 * @param field Field size in meters
 */
void
ConfigureGnbMobility(NodeContainer& gnbNodes, double field)
{
    std::cout << "=== Configuring gNB Positions ===" << std::endl;
    
    MobilityHelper gnbMob;
    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(200.0, 200.0, 30.0));
    gnbMob.SetPositionAllocator(gnbPos);
    gnbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMob.Install(gnbNodes);
    
    // Report gNB positions and distances
    std::vector<Vector> gnbPositions;
    for (uint32_t e = 0; e < gnbNodes.GetN(); ++e)
    {
        Ptr<MobilityModel> mm = gnbNodes.Get(e)->GetObject<MobilityModel>();
        Vector p = mm->GetPosition();
        gnbPositions.push_back(p);
        std::cout << "gNB" << e << ": (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    }
    
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "gNB pairwise distances (m):" << std::endl;
    for (uint32_t i = 0; i < gnbPositions.size(); ++i)
    {
        for (uint32_t j = i + 1; j < gnbPositions.size(); ++j)
        {
            double dx = gnbPositions[i].x - gnbPositions[j].x;
            double dy = gnbPositions[i].y - gnbPositions[j].y;
            double dz = gnbPositions[i].z - gnbPositions[j].z;
            double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            std::cout << "gNB" << i << "-gNB" << j << ": " << d << std::endl;
        }
    }
 
}

/**
 * Create building obstacles in the simulation area
 * @param field Field size in meters
 * @return Container with all buildings
 */
BuildingContainer
CreateBuildingObstacles(double field)
{
    std::cout << "=== Creating Building Obstacles ===" << std::endl;
    
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
  
    
    return buildings;
}

void
AttachUesToGnbs(Ptr<NrHelper> nrHelper,
               NodeContainer ueNodes,
               NodeContainer gnbNodes,
               NetDeviceContainer ueDevs,
               NetDeviceContainer gnbDevs)
{

    if (gnbNodes.GetN() > 1)
    {
        nrHelper->AddX2Interface(gnbNodes);
    }

    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(160)));

    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    std::map<uint16_t, std::pair<uint32_t, Vector>> cellIdToInfo;
    for (uint32_t g = 0; g < gnbNodes.GetN(); ++g)
    {
        Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(g));
        NS_ABORT_MSG_IF(gnbDev == nullptr, "AttachUesToGnbs: expected NrGnbNetDevice");
        uint16_t cellId = gnbDev->GetCellId();
        Vector pos = gnbNodes.Get(g)->GetObject<MobilityModel>()->GetPosition();
        cellIdToInfo.emplace(cellId, std::make_pair(g, pos));
    }

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> ueMob = ueNodes.Get(i)->GetObject<MobilityModel>();
        NS_ABORT_MSG_IF(ueMob == nullptr, "AttachUesToGnbs: UE " << i << " missing MobilityModel");
        Vector uePos = ueMob->GetPosition();

        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(i));
        NS_ABORT_MSG_IF(ueDev == nullptr, "AttachUesToGnbs: expected NrUeNetDevice");
        Ptr<NrUeRrc> ueRrc = ueDev->GetRrc();
        uint16_t servingCellId = ueRrc->GetCellId();

        uint32_t servingIndex = 0;
        Vector gnbPos;
        auto cellIt = cellIdToInfo.find(servingCellId);
        if (cellIt != cellIdToInfo.end())
        {
            servingIndex = cellIt->second.first;
            gnbPos = cellIt->second.second;
        }
        else
        {
            gnbPos = gnbNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
        }

        double dx = uePos.x - gnbPos.x;
        double dy = uePos.y - gnbPos.y;
        double dz = uePos.z - gnbPos.z;
        double groundDist = std::sqrt(dx * dx + dy * dy);
        double spatialDist = std::sqrt(dx * dx + dy * dy + dz * dz);

        std::cout << "UE " << i << " attached to gNB" << servingIndex << " (CellId=" << servingCellId
                  << ") | UE=(" << uePos.x << ", " << uePos.y << ", " << uePos.z << ") gNB=("
                  << gnbPos.x << ", " << gnbPos.y << ", " << gnbPos.z
                  << ") groundDist=" << std::fixed << std::setprecision(2) << groundDist
                  << "m spatialDist=" << spatialDist << "m" << std::endl;
    }

  
}

/**
 * Setup internet applications for all UEs
 * @param ueNodes Container of UE nodes
 * @param remoteHost Remote host node (internet server)
 * @param ueIpIfaces UE IP interfaces
 * @param simStop Simulation stop time
 */
void
SetupInternetApplications(NodeContainer ueNodes,
                              Ptr<Node> remoteHost,
                              Ipv4InterfaceContainer ueIpIfaces,
                              double simStop,
                              uint32_t packetSize,
                              uint32_t httpBytes,
                              uint32_t httpsBytes,
                              uint32_t videoBytes,
                              uint32_t voipBytes,
                              uint32_t mixedBytes,
                              uint32_t extraHttpBytes)
{
        
    
    Ipv4Address remoteHostAddr = remoteHost->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
  
    
    const uint16_t httpPort = 80;
    const uint16_t httpsPort = 443;
    const uint16_t videoPort = 8080;
    const uint16_t voipPort = 5060;
    const uint16_t dnsPort = 53;
    
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;
    
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
    
    UdpServerHelper dnsServer(dnsPort);
    serverApps.Add(dnsServer.Install(remoteHost));
    
    serverApps.Start(Seconds(0.5));
    serverApps.Stop(Seconds(simStop));
    
    enum class FlowType
    {
        Http,
        Https,
        Video,
        Voip,
        Mixed
    };
    
    struct FlowPattern
    {
        FlowType type;
        uint32_t maxBytes;
        double startTime;
    };
    
    const std::vector<FlowPattern> kFlowPattern = {
        {FlowType::Http,  httpBytes,       10.0},
        {FlowType::Http,  httpBytes,       10.1},
        {FlowType::Https, httpsBytes,      10.2},
        {FlowType::Https, httpsBytes,      10.3},
        {FlowType::Video, videoBytes,      10.4},
        {FlowType::Video, videoBytes,      10.5},
        {FlowType::Voip,  voipBytes,       10.6},
        {FlowType::Voip,  voipBytes,       10.7},
        {FlowType::Http,  extraHttpBytes,  10.8},
        {FlowType::Mixed, mixedBytes,      11.0},
    };
    
    const InetSocketAddress httpAddress(remoteHostAddr, httpPort);
    const InetSocketAddress httpsAddress(remoteHostAddr, httpsPort);
    const InetSocketAddress videoAddress(remoteHostAddr, videoPort);
    const InetSocketAddress voipAddress(remoteHostAddr, voipPort);
    
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        const uint32_t patternIndex = (i + kFlowPattern.size() / 2) % kFlowPattern.size();
        const FlowPattern& pattern = kFlowPattern[patternIndex];
        double cycleOffset = static_cast<double>(i / kFlowPattern.size());
        double startTime = pattern.startTime + cycleOffset;
        
        switch (pattern.type)
        {
        case FlowType::Http:
        {
            BulkSendHelper sender("ns3::TcpSocketFactory", httpAddress);
            sender.SetAttribute("MaxBytes", UintegerValue(pattern.maxBytes));
            sender.SetAttribute("SendSize", UintegerValue(packetSize));
            ApplicationContainer app = sender.Install(ueNodes.Get(i));
            app.Start(Seconds(startTime));
            app.Stop(Seconds(simStop));
            clientApps.Add(app);
            break;
        }
        case FlowType::Https:
        {
            BulkSendHelper sender("ns3::TcpSocketFactory", httpsAddress);
            sender.SetAttribute("MaxBytes", UintegerValue(pattern.maxBytes));
            sender.SetAttribute("SendSize", UintegerValue(packetSize));
            ApplicationContainer app = sender.Install(ueNodes.Get(i));
            app.Start(Seconds(startTime));
            app.Stop(Seconds(simStop));
            clientApps.Add(app);
            break;
        }
        case FlowType::Video:
        {
            BulkSendHelper sender("ns3::TcpSocketFactory", videoAddress);
            sender.SetAttribute("MaxBytes", UintegerValue(pattern.maxBytes));
            sender.SetAttribute("SendSize", UintegerValue(packetSize));
            ApplicationContainer app = sender.Install(ueNodes.Get(i));
            app.Start(Seconds(startTime));
            app.Stop(Seconds(simStop));
            clientApps.Add(app);
            break;
        }
        case FlowType::Mixed:
        {
            BulkSendHelper sender("ns3::TcpSocketFactory", httpAddress);
            sender.SetAttribute("MaxBytes", UintegerValue(pattern.maxBytes));
            sender.SetAttribute("SendSize", UintegerValue(packetSize));
            ApplicationContainer app = sender.Install(ueNodes.Get(i));
            app.Start(Seconds(startTime));
            app.Stop(Seconds(simStop));
            clientApps.Add(app);
            break;
        }
        case FlowType::Voip:
        {
            OnOffHelper client("ns3::UdpSocketFactory", voipAddress);
            client.SetAttribute("DataRate", DataRateValue(DataRate("1.5Mbps")));
            client.SetAttribute("PacketSize", UintegerValue(packetSize));
            client.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
            client.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
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
