/**
 \file restricted_common_nodes.h
 \brief Header file for restricted common nodes.

 \author              Babarczi Peter
 \author              babarczi@tmit.bme.hu
 \date                2015 november
*/

#ifndef RESTRICTED_COMMON_NODES_H
#define RESTRICTED_COMMON_NODES_H

#include <lemon_routing/inverse_capacity_proportion.h>
#include "mwld_simulator.h"

#include <lemon_routing/srg_set.h>
#include <lemon/connectivity.h>
#include <lemon_routing/make_names.h>
#include "random.h"
#include <sstream>

#include <lemon/lp.h>
#include <lemon/lp_base.h>
#include <set>

#include <sys/time.h>
#include <sys/resource.h>
#include <lemon/time_measure.h>

typedef SplitNodes<Digraph> SplitGraph;
typedef SplitGraph::CombinedArcMap<DCostMap, DNodeCostMap> CombinedCostMap;
typedef SplitGraph::CombinedArcMap<DBoolMap, DNodeBoolMap> CombinedBoolMap;
typedef FilterArcs<SplitGraph, CombinedBoolMap> DFilteredSplitRSPGraph;
typedef Digraph::ArcMap<set<int> > ArcSetMap;

using namespace lemonROUTING;
using namespace std;

//! Main class containing all problem variants for the MWLD problem, inlcuding SCN and SP as well.
class RestrictedCommonNodes : public virtual MWLDSimulator{
public:
	DNodeCostMap node_cost;
	DNodeCostMap node_delay;
	DNodeBoolMap on_working;
	DNodeCostMap node_visible;
	DNodeBoolMap type_A_node;
	DNodeBoolMap type_A_allowed_common_node;

	DCostMap delay;
	DCostMap visible;
	DBoolMap transformed_edge;
	DBoolMap transformed_visible_edge;
	DBoolMap non_working_edge;

	DIntMap edge_id;

	ArcSetMap original_edges;

	list<TrafficDemand<Digraph> > built_connections;

	//For statistics
	double total_solcost;
	int node_num;
	
	int min_common_nodes;
	int max_common_nodes;
	int type_A_nodes;
	double avg_common_nodes;
	int connections_with_benefits;
	int min_cost_connections;
	int improvable_connections;
	int allowing_common_nodes_leads_to_optimum;
	double mwld_improvement_to_node_disjoint;
	double link_disjoint_improvement_to_node_disjoint;
	double link_disoint_cost;
	double node_disoint_cost;
	double avg_wp_length;
	int maximal_edge_weight;


	//! Create the simulator object 
	RestrictedCommonNodes(Digraph & _g): MWLDSimulator(_g), edge_id(_g), delay(_g), visible(_g), transformed_edge(_g), transformed_visible_edge(_g), non_working_edge(_g), node_cost(_g), node_delay(_g), on_working(_g), node_visible(_g), original_edges(_g), type_A_node(_g), type_A_allowed_common_node(_g) {
		built_connections.clear();
		total_solcost = 0.0; 
		min_common_nodes = 10000;
		max_common_nodes = 0;
		avg_common_nodes = 0.0;
		type_A_nodes = 0;
		connections_with_benefits = 0;
		mwld_improvement_to_node_disjoint = 0.0;
		link_disjoint_improvement_to_node_disjoint = 0.0;
		link_disoint_cost = 0.0;
		node_disoint_cost = 0.0;
		min_cost_connections = 0;
		improvable_connections = 0;
		allowing_common_nodes_leads_to_optimum = 0;
		avg_wp_length = 0.0;
		node_num = 0;
		maximal_edge_weight = 0;
	}

	~RestrictedCommonNodes(){};

	// MWLD problem (ICNP upper bound algorithm)
	// Stage 1: Create Transformed Graph; Stage 2: Calculate Restricted Shortest Path; Stage 3: Save shortest pairs of link-disjoint paths in the original graph
	int initialize_MWLD_maps(double type_A_percentage, double type_A_allowed_common_percentage);
	void calculateMWLD(int max_common_node_num, int method, rusage &MWLDStage1Creation);
	void createTransformedGraph(SplitGraph &splitnodes, CombinedCostMap &combined_cost, CombinedCostMap &combined_delay, CombinedCostMap &combined_visible);
	
	// Restrcited Conditional Second Shortest Path (ToN)
	// Stage 1: Calculate shortest path, remove its links; Stage 2: Run modified Bellmann Ford
	void calculateConditionalSecondPath(int max_common_node_num, int method);
};

#endif 


