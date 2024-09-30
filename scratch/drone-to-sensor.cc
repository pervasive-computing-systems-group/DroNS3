/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015, IMDEA Networks Institute
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
 *
 * Author: Jonathan Diller <jonathan.a.diller@gmail.com>
.*
 * This program emulates a drone collecting data from a sensor on the ground.
 * We use TCP with the Friis propagation loss model and the YANS WiFi and
 * Error-Rate models. The basic setup is shown below.
 *
 * Network topology:
 *
 *    Ap <---distance---> STA
 *    *                    *
 *    |                    |
 *   n0                    n1
 * (drone)              (sensor)
 *
 * The drone and the sensor are "distance" apart (in meters). The sensor
 * attempts to transmit a payload of data as fast as possible. The size
 * of the payload, the distance between the drone and the sensor, and
 * the TX power of the sensor can all be set using command line arguments.
 * The program will delay termination for the duration that the simulator
 * decided was required to transmit the payload. This behavior can be
 * disabled by seeing the delay argument to false.
 *
 * Example run:
 *
 * ./ns3 run scratch/drone-to-sensor --command-template="%s --distance=30 --payload=10000000--txpower=5 --delay=true"
 *
 */

#include <chrono>
#include <thread>

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/tcp-westwood.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/packet-sink.h"

#include <cmath>

NS_LOG_COMPONENT_DEFINE ("wifi-tcp");

using namespace ns3;

Time lastRX;

// Packet sink received-packet callback
static void ReceivePacket(std::string path, Ptr<const Packet> packet, const Address &from) {
	// Mark when we received the packet
	lastRX = Simulator::Now ();
}


int main (int argc, char *argv[]) {
	exit(-1)
	// Record time to run simulation
	std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();

	// Volume of data to send (bytes)
	uint32_t payload = 1024;
	// Distance between nodes (in meters)
	double distance = 50;
	// TX power of sensor sending data (dBm)
	double txPower = 10;
	// Allow the program to delay the realistic duration for data communication
	bool delay = true;
	// Total time to let simulation run (must be larger than time required to move data)
	double simulationTime = 50;

	// Command line argument parser setup.
	CommandLine cmd (__FILE__);
	cmd.AddValue ("distance", "meters separation between nodes", distance);
	cmd.AddValue ("payload", "Bytes to send", payload);
	cmd.AddValue ("txpower", "TX power of sender in dBm", txPower);
	cmd.AddValue ("delay", "Delay execution for realistic duration for data communication", delay);
	cmd.Parse (argc, argv);

	/*
	 * Available Transport protocols: TcpNewReno, TcpHybla, TcpHighSpeed, TcpHtcp, TcpVegas,
	 * TcpScalable, TcpVeno, TcpBic, TcpYeah, TcpIllinois, TcpWestwood, TcpWestwoodPlus, TcpLedbat
	 *
	 * Do we care? Use default?
	 */
//	Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName ("ns3::TcpNewReno")));

	// Create an access point node (the drone) and a station node (the sensor)
	NodeContainer networkNodes;
	networkNodes.Create (2);
	Ptr<Node> apWifiNode = networkNodes.Get (0);
	Ptr<Node> staWifiNode = networkNodes.Get (1);

	/* Set up Legacy Channel */
	YansWifiChannelHelper wifiChannel;
	wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5.180e9));

	/* Setup Physical Layer */
	YansWifiPhyHelper wifiPhy;
	wifiPhy.SetChannel (wifiChannel.Create ());
	wifiPhy.SetErrorRateModel ("ns3::YansErrorRateModel");
	wifiPhy.Set ("ChannelSettings", StringValue ("{0, 20, BAND_2_4GHZ, 0}"));

	WifiMacHelper wifiMac;
	WifiHelper wifiHelper;
	wifiHelper.SetStandard (WIFI_STANDARD_80211n);

	/* Configure AP */
	Ssid ssid = Ssid ("network");
	wifiMac.SetType ("ns3::ApWifiMac",
	"Ssid", SsidValue (ssid));
	wifiPhy.Set ("TxPowerStart", DoubleValue (10.0)); // dBm (1.26 mW)
	wifiPhy.Set ("TxPowerEnd", DoubleValue (10.0));

	NetDeviceContainer apDevice;
	apDevice = wifiHelper.Install (wifiPhy, wifiMac, apWifiNode);

	/* Configure STA */
	wifiMac.SetType ("ns3::StaWifiMac",
	"Ssid", SsidValue (ssid));
	wifiPhy.Set ("TxPowerStart", DoubleValue (txPower)); // dBm (1.26 mW)
	wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));

	NetDeviceContainer staDevices;
	staDevices = wifiHelper.Install (wifiPhy, wifiMac, staWifiNode);

	/* Mobility model */
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
	positionAlloc->Add (Vector (0.0, 0.0, 0.0));
	positionAlloc->Add (Vector (distance, 0.0, 0.0));

	mobility.SetPositionAllocator (positionAlloc);
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (apWifiNode);
	mobility.Install (staWifiNode);

	/* Internet stack */
	InternetStackHelper stack;
	stack.Install (networkNodes);

	Ipv4AddressHelper address;
	address.SetBase ("10.0.0.0", "255.255.255.0");
	Ipv4InterfaceContainer apInterface;
	apInterface = address.Assign (apDevice);
	Ipv4InterfaceContainer staInterface;
	staInterface = address.Assign (staDevices);

	/* Populate routing table */
	Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

	// Create a BulkSendApplication and install it on node 0
	uint16_t port = 9;  // well-known echo port number

	BulkSendHelper source ("ns3::TcpSocketFactory",
	InetSocketAddress (apInterface.GetAddress (0), port));
	// Set the amount of data to send in bytes.  Zero is unlimited.
	source.SetAttribute ("MaxBytes", UintegerValue (payload));
	ApplicationContainer sourceApps = source.Install (staWifiNode);
	sourceApps.Start (Seconds (0.0));

	// Create a PacketSinkApplication and install it on node 1
	PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
	ApplicationContainer sinkApps = sinkHelper.Install (apWifiNode);

	//Register packet receptions to calculate throughput
	Config::Connect ("/NodeList/0/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&ReceivePacket));

	sinkApps.Start (Seconds (0.0));

	/* Start Simulation */
	Simulator::Stop (Seconds (simulationTime));
	Simulator::Run ();

	Simulator::Destroy ();

	Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (sinkApps.Get (0));
	std::cout << "Total Bytes Received: " << sink1->GetTotalRx() << std::endl << lastRX.GetSeconds() << "s " << std::endl;


	// Cody and ava changes

	// set amount of time the data takes to transfer, based on distance and amount of data
	float data_transfer_rate;
	float MAX_TRANSFER_RATE = 0.2; // MBPS
	float coefficient = 0.5; // found from emperaiacal testing

	data_transfer_rate = coefficient / sqrtf(distance);

	// add some gauussian distrubtion 
	// https://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	std::random_device rd{};
    std::mt19937 gen{rd()};
 
    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution d{data_transfer_rate, 0.86934};
 
    // draw a sample from the normal distribution and round it to an integer
    data_transfer_rate = [&d, &gen]{ return d(gen); };

	if (data_transfer_rate > MAX_TRANSFER_RATE) {
		data_transfer_rate = MAX_TRANSFER_RATE;
	}
	if (data_transfer_rate <= 0) {
		std::cout << "NS3 Failed to connect (speed below zero)\n";
		exit(1);
	}

	int64_t time_delay_ms = 1000 * payload / data_transfer_rate;
	

	if(delay) {
		// Record end of simulation time
		std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();

		// Determine elapsed time
		int64_t time_lapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - beginTime).count();

		// We have successfully contacted the node. Sleep to simulate data collection
		std::this_thread::sleep_for(std::chrono::milliseconds(time_delay_ms - time_lapsed));
	}

	// Check to see if any data was collected
	if(sink1->GetTotalRx() <= 0) {
		// No data collected. Setting payload = 0 would cause an endless stream
		// of data, therefore the sensor failed to connect to the drone.
		std::cout << "Failed to connect\n";
		// Hard fail so that user knows that data transmission failed
		exit(3);
	}

	return 0;
}
