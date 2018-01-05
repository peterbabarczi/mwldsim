/**
 \file  demo_restricted.cpp
 \brief Calculates paths with restricted common nodes.

Usage:

./demo_fdp [--help|-h|-help] [-cap_param int] [-comment str] [-global_optimum] [-php] [-xml] [lgf file] [srg file] [trf file]

Where:

[lgf file]
     The input topology

[srg file]
     The list of shared risk link groups

[trf file]
     The list of traffic demands

--help|-h|-help
     Print a short help message


 \author              Babarczi Peter
 \author              babarczi@tmit.bme.hu
 \date                2015 november
*/

#include "restricted_common_nodes.h"
#include <lemon_routing/php_arg_parser.h>
#include <iostream>
#include <sstream>
#include <string>
#include <lemon/time_measure.h>


unsigned logging_priority_level=1;  
unsigned number_of_errors=0;  
const bool LOG_MSG=true;  
int php_level[6]= { 1,1,1,1,1,1 };  

//! Running time calculation
string timeDif(const rusage& _start, const rusage& _end)
{
	double time = (double)(_end.ru_utime.tv_usec-_start.ru_utime.tv_usec)/1000000
			+(double)(_end.ru_utime.tv_sec-_start.ru_utime.tv_sec)
			+(double)(_end.ru_stime.tv_sec-_start.ru_stime.tv_sec)
			+(double)(_end.ru_stime.tv_usec-_start.ru_stime.tv_usec)/1000000;
	char buff[256] = {'\0'};
	sprintf(buff,"%f",time);
	return buff;
}


int main(int argc, char** argv){
	if (argc<=1) {
		error_msg<<"Usage: "<<argv[0]<<" --help \n";
		return -1;
	}

	PhpArgParser ap(argc,argv);
	log_msg_start;
	string comment;

	ap.boolOption("xml","Results are stored in an xml file")	 	 
	.refOption("comment", "Additional comment to the output file name",comment)	
	.intOption("cap_param","The capacity of the edges", 32)
	.intOption("method","The applied algorithm: 11 - MWLD RSP (ICNP), 12 - MWLD Bellman-Ford, (ICNP), 21 - Conditional Second Shortest Path (ToN) Upper, 22 - CSSP (ToN) Tight, 23 CSSP (ToN) Lower, 31 - Selective Common Nodes (ToN) RSP, 32 - Selective Common Nodes (ToN) Bellman-Ford", 1)
	.intOption("max_common_node_num","The maximum allowed common nodes (upper bound) in the somwhow-disjont paths", 0)
	.doubleOption("type_A_percentage","The percentage of type A nodes", 0.0)
	.doubleOption("type_A_allowed_common_percentage","The percentage of allowed common type A nodes", 1.1)
	.other("[lgf file]","The input topology")
	.other("[srg file]","The list of shared risk link groups")
	.other("[trf file]","The list of traffic demands");

	ap.throwOnProblems ();
	try {
		ap.run();
	} catch (ArgParserException &) { 
		error_msg<<"Parsing command line error:"; 
		return false;
	}

	string lgf_file_name=ap.files()[0];
	string srg_file_name=ap.files()[1];
	string trf_file_name=ap.files()[2];

	// We read the directed graph from the .srg file.
	ifstream lgf_file(srg_file_name.c_str());
	if (!lgf_file) {
		error_msg<<"Error opening lgf file of "<<lgf_file_name;
		return false;
	}

	ifstream srg_file(srg_file_name.c_str());
	if (!srg_file) {
		error_msg<<"Error opening file .srg\n";
	return false;
	}

	ifstream trf_file(trf_file_name.c_str());
	if (!trf_file) {
		error_msg<<"Error opening file .trf\n";
		return false;                   // abnormal exit: error opening file
	}

	int method = ap["method"];
	int capacity = ap["cap_param"];
	int max_common_node_num = ap["max_common_node_num"];
	double type_A_percentage = ap["type_A_percentage"];
	bool int_xml = ap["xml"];
	double type_A_allowed_common_percentage;
	if(type_A_allowed_common_percentage	> 1) type_A_allowed_common_percentage = type_A_percentage;
	else type_A_allowed_common_percentage = ap["type_A_allowed_common_percentage"];

	if(type_A_allowed_common_percentage > type_A_percentage){
		cout <<"ERROR: program called with false type_A_percentage and type_A_allowed_common_percentage values, CORRECT: type_A_percentage >= type_A_allowed_common_percentage" << endl;
		return -1;
	}


	Digraph g;
	RestrictedCommonNodes alg(g);	
	
	rusage enviromentBuildStartTime;
	getrusage(RUSAGE_SELF, &enviromentBuildStartTime);
	
	alg.readGraph(lgf_file, capacity);
	alg.readSRLGSet(srg_file);
	alg.readTraffic(trf_file);

	rusage enviromentBuildEndTime;
	getrusage(RUSAGE_SELF, &enviromentBuildEndTime);

	rusage MWLDStage1Creation;	
	
	// Initialize maps with the required number of Type A nodes (quit if not possible)
	int init_ret_val = 0;
	init_ret_val = alg.initialize_MWLD_maps(type_A_percentage, type_A_allowed_common_percentage);
	if(init_ret_val < 0) return -1;	
		
	if(method == 11 || method == 12 || method == 31 || method == 32){
		// Run the simulations.
		alg.calculateMWLD(max_common_node_num, method, MWLDStage1Creation);
	}
	else if(method == 21 || method == 22 || method == 23){
		alg.calculateConditionalSecondPath(max_common_node_num, method);
	}
	else{
		cout << "Selected method does not exists!" << endl;
		return -1;
	}


	rusage currenttime;
	getrusage(RUSAGE_SELF, &currenttime);

	// Save the results.
	string runTime = timeDif(enviromentBuildEndTime, currenttime);
	string buildTime = timeDif(enviromentBuildStartTime, enviromentBuildEndTime);
	string graphCreationTime = timeDif(enviromentBuildEndTime, MWLDStage1Creation);	
	string demandCalcTime = timeDif(MWLDStage1Creation, currenttime);


	ostringstream out_file_name;
	if (int_xml) out_file_name << lgf_file_name.substr(0, lgf_file_name.size() - 4) << comment;
	else out_file_name << lgf_file_name.substr(0, lgf_file_name.size() - 4);

	out_file_name << ".xml";
	ofstream os(out_file_name.str().c_str());
	if (!os)
	{
		error_msg << "Error opening file " << out_file_name;
		exit(-1); // abnormal exit: error opening file
	}
	os << "<simulator>\n";
	if(method == 11)	os << "\t<ProtectionMethod>" << "mwld-rsp" << "</ProtectionMethod>\n";
	else if(method == 12)	os << "\t<ProtectionMethod>" << "mwld-bf" << "</ProtectionMethod>\n";
	else if(method == 21) os << "\t<ProtectionMethod>" << "cssp-upper" << "</ProtectionMethod>\n";
	else if(method == 22) os << "\t<ProtectionMethod>" << "cssp-tight" << "</ProtectionMethod>\n";
	else if(method == 23) os << "\t<ProtectionMethod>" << "cssp-lower" << "</ProtectionMethod>\n";
	else if(method == 31) os << "\t<ProtectionMethod>" << "scn-rsp" << "</ProtectionMethod>\n";
	else if(method == 32) os << "\t<ProtectionMethod>" << "scn-bf" << "</ProtectionMethod>\n";
	os << "\t<Network>\n";
	os << "\t\t<NodeNum>" << alg.node_number << "</NodeNum>\n";
	os << "\t\t<EdgeNum>" << alg.edge_number << "</EdgeNum>\n";
	os << "\t\t<SRLGNum>" << alg.srg_number << "</SRLGNum>\n";
	os << "\t\t<DemandNum>" << alg.demand_number << "</DemandNum>\n";
	os <<"\t\t<AllowedCommonNodes>" << max_common_node_num  << "</AllowedCommonNodes>\n";	
	os <<"\t\t<Deg4Nodes>" << alg.type_A_nodes  << "</Deg4Nodes>\n";
	os <<"\t\t<TypeAPercenatage>" << type_A_percentage  << "</TypeAPercenatage>\n";	
	os <<"\t\t<TypeAAllowedPercenatage>" << type_A_allowed_common_percentage << "</TypeAAllowedPercenatage>\n";	
	os << "\t</Network>\n";
	os << "\t<Statistics>\n";
	os << "\t\t<NodeDisjointAvgCost>" << (double)alg.node_disoint_cost / (double)(alg.demand_number
			- alg.blocked_connections) << "</NodeDisjointAvgCost>\n";
	os << "\t\t<MWLDAvgCost>" << (double)alg.total_solcost / (double)(alg.demand_number
			- alg.blocked_connections) << "</MWLDAvgCost>\n";
	os << "\t\t<LinkDisjointAvgCost>" << (double)alg.link_disoint_cost / (double)(alg.demand_number
			- alg.blocked_connections) << "</LinkDisjointAvgCost>\n";
	os <<"\t\t<RelativeImprovement>" << (((double) alg.link_disjoint_improvement_to_node_disjoint > 0.0) ? (double) alg.mwld_improvement_to_node_disjoint/(double) alg.link_disjoint_improvement_to_node_disjoint : (0.0)) << "</RelativeImprovement>\n";
	os <<"\t\t<OptimalSTPairs>" << (double) alg.min_cost_connections / (double)(alg.demand_number
			- alg.blocked_connections)  << "</OptimalSTPairs>\n";
	os <<"\t\t<ImprovableConnections>" << alg.improvable_connections << "</ImprovableConnections>\n";
	os <<"\t\t<ImprovableConnectionsPercentage>" << (double)alg.improvable_connections/(double)alg.demand_number << "</ImprovableConnectionsPercentage>\n";
	os <<"\t\t<ImprovedConnectionsPercentage>" << (((double)alg.improvable_connections  > 0.0) ? (double) alg.connections_with_benefits/ (double)alg.improvable_connections : (0.0)) << "</ImprovedConnectionsPercentage>\n";
	os <<"\t\t<ImprovedConnectionsToOptimumPercentage>" << (((double)alg.improvable_connections > 0.0) ? ((double) alg.allowing_common_nodes_leads_to_optimum / (double)alg.improvable_connections) : (0.0)) << "</ImprovedConnectionsToOptimumPercentage>\n";
	os <<"\t\t<MaxMWLDCommonNodesPerConnection>" << alg.max_common_nodes  << "</MaxMWLDCommonNodesPerConnection>\n";
	os << "\t\t<MWLDTotalSolutionCost>" << alg.total_solcost  << "</MWLDTotalSolutionCost>\n";	
	os << "\t\t<UsedEdgeNumber>" << alg.used_edge_number/2 << "</UsedEdgeNumber>\n";
	// As we have unidirectional connections, we divide it by 2 (as used_edges counts the links in both directions)
	os << "\t\t<AvgEdgeNumber>" << alg.used_edge_number/2.0 / (double)(alg.demand_number
			- alg.blocked_connections) << "</AvgEdgeNumber>\n";
	os << "\t\t<AvgWPLength>" << alg.avg_wp_length / (double)(alg.demand_number
			- alg.blocked_connections) << "</AvgWPLength>\n";
	os << "\t\t<TotalBlockedConnections>" << alg.blocked_connections
			<< "</TotalBlockedConnections>\n";
	os << "\t\t<BlockingProbability>" << (double) alg.blocked_connections
			/ (double) alg.demand_number << "</BlockingProbability>\n";
	os << "\t\t<TotalRunningTime>" << runTime << "</TotalRunningTime>\n";
	os << "\t\t<AvgRunningTime>" << atof(runTime.c_str()) / (double)alg.demand_number << "</AvgRunningTime>\n";
	os << "\t\t<TotalBuildTime>" << buildTime << "</TotalBuildTime>\n";
	os << "\t\t<AvgCalculationTime>" << atof(demandCalcTime.c_str()) / (double)alg.demand_number << "</<AvgCalculationTime>\n";
	os << "\t\t<TotalGraphCreationTime>" << graphCreationTime << "</TotalGraphCreationTime>\n";
	os << "\t</Statistics>\n";
	os << "</simulator>\n";
	os.close();

	lgf_file.close();
	srg_file.close();
	trf_file.close();

	log_msg_finish;
	return 0;
};

