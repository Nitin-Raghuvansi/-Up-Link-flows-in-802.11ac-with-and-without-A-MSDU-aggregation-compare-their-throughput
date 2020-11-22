/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Sébastien Deronne
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
 * Author: Sébastien Deronne <sebastien.deronne@gmail.com>
 */

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac.h"
#include<iostream>
// This is an example that illustrates how 802.11n aggregation is configured.
// It defines 4 independent Wi-Fi networks (working on different channels).
// Each network contains one access point and one station. Each station
// continuously transmits data packets to its respective AP.
//
// Network topology (numbers in parentheses are channel numbers):
//
//  Network A (36)   Network B (40)   Network C (44)   Network D (48)
//   *      *          *      *         *      *          *      *
//   |      |          |      |         |      |          |      |
//  AP A   STA A      AP B   STA B     AP C   STA C      AP D   STA D
//
// The aggregation parameters are configured differently on the 4 stations:
// - station A uses default aggregation parameter values (A-MSDU disabled, A-MPDU enabled with maximum size of 65 kB);
// - station B doesn't use aggregation (both A-MPDU and A-MSDU are disabled);
// - station C enables A-MSDU (with maximum size of 8 kB) but disables A-MPDU;
// - station D uses two-level aggregation (A-MPDU with maximum size of 32 kB and A-MSDU with maximum size of 4 kB).
//
//Packets in this simulation belong to BestEffort Access Class (AC_BE).
//
// The user can select the distance between the stations and the APs and can enable/disable the RTS/CTS mechanism.
// Example: ./waf --run "wifi-aggregation --distance=10 --enableRts=0 --simulationTime=20"
//
// The output prints the throughput measured for the 4 cases/networks described above. When default aggregation parameters are enabled, the
// maximum A-MPDU size is 65 kB and the throughput is maximal. When aggregation is disabled, the throughput is about the half of the physical
// bitrate. When only A-MSDU is enabled, the throughput is increased but is not maximal, since the maximum A-MSDU size is limited to 7935 bytes
// (whereas the maximum A-MPDU size is limited to 65535 bytes). When A-MSDU and A-MPDU are both enabled (= two-level aggregation),
// the throughput is slightly smaller than the first scenario since we set a smaller maximum A-MPDU size.
//
// When the distance is increased, the frame error rate gets higher, and the output shows how it affects the throughput for the 4 networks.
// Even through A-MSDU has less overheads than A-MPDU, A-MSDU is less robust against transmission errors than A-MPDU. When the distance is
// augmented, the throughput for the third scenario is more affected than the throughput obtained in other networks.

using namespace ns3;
using namespace std;
NS_LOG_COMPONENT_DEFINE ("SimpleMpduAggregation");

int main (int argc, char *argv[])
{
  uint32_t payloadSize = 1472; //bytes
  double simulationTime = 10; //seconds
  double distance = 5; //meters
  bool enableRts = 0;
  bool enablePcap = 0;
  bool verifyResults = 0; //used for regression

  CommandLine cmd (__FILE__);
  cmd.AddValue ("payloadSize", "Payload size in bytes", payloadSize);
  cmd.AddValue ("enableRts", "Enable or disable RTS/CTS", enableRts);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("distance", "Distance in meters between the station and the access point", distance);
  cmd.AddValue ("enablePcap", "Enable/disable pcap file generation", enablePcap);
  cmd.AddValue ("verifyResults", "Enable/disable results verification at the end of the simulation", verifyResults);
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", enableRts ? StringValue ("0") : StringValue ("999999"));

  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (30);
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("HtMcs7"), "ControlMode", StringValue ("HtMcs0"));
  WifiMacHelper mac;

  NetDeviceContainer staDevice,apDevice;
  Ssid ssid;

  // Network A
  ssid = Ssid ("ns3-80211ac");
  //phy.Set ("ChannelNumber", UintegerValue (36));
  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid));
  staDevice = wifi.Install (phy, mac, wifiStaNodes);
  
  
  // Disable A-MPDU and enable A-MSDU
  
  for(uint16_t i=0;i<30;i++)
  {
  Ptr<NetDevice> dev = wifiStaNodes.Get (i)->GetDevice (0);
  Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (dev);
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (0));
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (0));
  }
  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid),
               "EnableBeaconJitter", BooleanValue (false));
  apDevice = wifi.Install (phy, mac, wifiApNodes);
  
  // Disable A-MPDU and  enable A-MSDU
  Ptr<NetDevice> dev = wifiApNodes.Get (0)->GetDevice (0);
  Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (dev);
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (0));
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (0));
  
  


  // Setting mobility model
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Set position for APs
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  
  // Set position for STAs

  for(uint16_t i=0;i<30;i++)
  {
  positionAlloc->Add (Vector (i+distance, 0.0, 0.0));
  
  }

  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (wifiApNodes);
  mobility.Install (wifiStaNodes);

  // Internet stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);

  Ipv4AddressHelper address;
  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer StaInterface;
  StaInterface= address.Assign (staDevice);
  Ipv4InterfaceContainer ApInterface;
  ApInterface = address.Assign (apDevice);

 


  // Setting applications
  ApplicationContainer serverApp;
  uint16_t port = 9;
  UdpServerHelper server (port);
  serverApp= server.Install ( wifiApNodes.Get (0));
  serverApp.Start (Seconds (0.0));
  serverApp.Stop (Seconds (simulationTime + 1));

  for(uint16_t i=0;i<30;i++)
  {
  UdpClientHelper client (ApInterface.GetAddress (0), port);
  client.SetAttribute ("MaxPackets", UintegerValue (1000));
  client.SetAttribute ("Interval", TimeValue (Time ("0.001"))); //packets/s
  client.SetAttribute ("PacketSize", UintegerValue (1024));
  ApplicationContainer clientApp = client.Install (wifiStaNodes.Get (i));
  clientApp.Start(Seconds (1.0));
  clientApp.Stop (Seconds (simulationTime + (1+(0.001*i))));

  }



 

  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();

  // Show results
  
  
  
   double totalPacketsThrough= DynamicCast<UdpServer> (serverApp.Get (0))->GetReceived ();
 

  Simulator::Destroy ();

 
  
  double throughput = totalPacketsThrough * payloadSize * 8 / (simulationTime * 1000000.0);
  std::cout << "Throughput with A-MPDU disabled and A-MSDU disabled (8kB): " << throughput << " Mbit/s" << '\n';
  if (verifyResults && (throughput < 51 || throughput > 52))
    {
      NS_LOG_ERROR ("Obtained throughput " << throughput << " is not in the expected boundaries!");
      exit (1);
    }
  
  

  return 0;
}
