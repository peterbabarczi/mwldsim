/**
\file mwld_simulator.h

 \author              Babarczi Peter
 \author              babarczi@tmit.bme.hu
 \date                2015 november
*/

#ifndef MWLD_SIMULATOR_H
#define MWLD_SIMULATOR_H

#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>

#include <sys/time.h>
#include <sys/resource.h>

#include <lemon/core.h>
#include <lemon/smart_graph.h>
#include <lemon/dim2.h>
#include <lemon/bfs.h>

#include <lemon_routing/make_names.h>
#include <lemon_routing/srg_set.h>
#include <lemon_routing/per_flow.h>
#include <lemon_routing/inverse_capacity_proportion.h>

#include <lemon_routing/php_arg_parser.h>
#include <lemon/time_measure.h>

#include <lemon/lp.h>
#include <lemon/lp_base.h>
//#include <lemon/cplex.h>
#include <lemon/gurobi.h>

#include "lemon/suurballe.h"
#include "lemon/dijkstra.h"
#include "lemon/path.h"
#include <lemon/random.h>
#include <lemon/bellman_ford.h>

using namespace lemonROUTING;
using namespace lemon;
using namespace std;

typedef SmartDigraph Digraph; 
typedef Digraph::ArcIt ArcIt; 
typedef Digraph::Arc Arc; 
typedef Digraph::NodeIt DNodeIt; 
typedef Digraph::Node DNode; 
typedef Digraph::OutArcIt OutArcIt; 
typedef Digraph::InArcIt InArcIt;  

typedef Digraph::ArcMap<double> DDoubleMap;  
typedef Digraph::ArcMap<int> DIntMap; 
typedef Digraph::ArcMap<bool> DBoolMap; 
typedef Digraph::NodeMap<int> DNodeIntMap; 
typedef Digraph::NodeMap<bool> DNodeBoolMap; 

typedef double UnavValue; 
typedef Digraph::ArcMap<UnavValue> ArcUnavMap;  
typedef Digraph::NodeMap<UnavValue> DNodeUnavMap; 

typedef double CostValue;
typedef Digraph::ArcMap<CostValue> DCostMap; 
typedef Digraph::NodeMap<CostValue> DNodeCostMap;

typedef double DCapValue; 
typedef Digraph::ArcMap<DCapValue> DCapMap; 
typedef Digraph::ArcMap<DCapValue> DFreeCap; 

typedef RestoreableSimpleSRGSet<Digraph> SRLGSet; 
typedef SRLGSet::SRG SRLG; 
typedef SRLGSet::SRGIt SRLGIt; 
typedef SRLGSet::SRGArcIt SRLGArcIt; 
typedef SRLGSet::SRGForArcIt SRLGForArcIt;  
typedef SRGSetReader<Digraph, SRLGSet> SRLGReader; 

typedef SRLGSet::SRGMap<long double> DoubleSRLGMap;
typedef SRLGSet::SRGMap<bool> BoolSRLGMap; 
typedef SRLGSet::SRGMap<int> IntSRLGMap; 
typedef SRLGSet::SRGMap<set<int> > IntSetSRLGMap; 
typedef SRLGSet::SRGMap<set<string> > StringSetSRLGMap; 
typedef SRLGSet::SRGMap<map<string,int > > StringIntMapSetSRLGMap; 
typedef Digraph::ArcMap<vector<string> > SRLGSetMap; 
typedef Digraph::NodeMap<set<int> > DNodeSRLGSetMap; 
typedef Digraph::NodeMap<set<string> > DNodeEdgeSetMap;
typedef Digraph::NodeMap<set<int> > DNodeProactableWorkingPathMap;

typedef dim2::Point<double> Point; 
typedef Digraph::NodeMap<Point> DCoordMap; 
typedef InverseCapacityProportion<Arc, double, DDoubleMap, DDoubleMap > DCostMap1; 

typedef FilterArcs<Digraph, DBoolMap> DFilteredGraph; 
typedef FilterArcs<DFilteredGraph, DBoolMap> FilteredSRLGGraph; 
typedef Dijkstra<FilteredSRLGGraph, DCostMap> FSRLGDijkstra; 
typedef Path<DFilteredGraph> DFPath; 
typedef PathNodeIt<DFPath> DFPathNodeIt; 
typedef Dijkstra<DFilteredGraph, DCostMap> DFDijkstra; 
typedef Dijkstra<DFilteredGraph, DCostMap1> DInvDijkstra; 
typedef DFilteredGraph::NodeMap<int> DFNodeIntMap; 
typedef DFilteredGraph::NodeMap<bool> DFNodeBoolMap; 
typedef Path<Digraph> DPath;

typedef FilterArcs<const DFilteredGraph, DBoolMap> DFFilteredGraph;
typedef Path<DFFilteredGraph> DFFPath; 
typedef Dijkstra<DFFilteredGraph, DCostMap> DFFDijkstra; 
typedef Dijkstra<DFFilteredGraph, DCostMap1> DFInvDijkstra; 

typedef Bfs<Digraph> DBFS; 
typedef Bfs<DFilteredGraph> DFilteredBFS; 

typedef GurobiMip Mip;
typedef DFilteredGraph::ArcMap<Mip::Col> LPVariableMap; 
typedef FilteredSRLGGraph::ArcMap<Mip::Col> LPVariableMapSRLG; 

	enum ConnType{BUILD=0, 
		RELEASE=1 
	};

	template <class GR> class TrafficDemand{
		typedef typename GR::Node GRNode; 
	private:
		const GR* g; 
		const SRLGSet* srg_set;  
	public:
		BoolSRLGMap affected; 
		ConnType connType; 
		int connId; 
		GRNode sourceNode; 
		GRNode targetNode; 
		DCapValue capacity; 
		DBoolMap solution_edge; 
		map<string, double> used_edges; 

		void ResetDemand();
		TrafficDemand(const GR* _g, const SRLGSet* _set,  ConnType _connType, int _connId, GRNode _sourceNode, GRNode _targetNode, DCapValue _capacity);
		TrafficDemand(const TrafficDemand& _other);
		const TrafficDemand<GR>& operator=(const TrafficDemand<GR>& _other);
	};

template <class GR> void TrafficDemand<GR>::ResetDemand()
	{
		for (ArcIt eit(*g); eit != INVALID; ++eit)
			solution_edge[eit] = false;

		for (SRLGIt it(*srg_set); it != INVALID; ++it)
			affected[it] = false;

		used_edges.clear();
	};


template <class GR> 	TrafficDemand<GR>::TrafficDemand(const GR* _g, const SRLGSet* _set, ConnType _connType,
			int _connId, typename GR::Node _sourceNode, typename GR::Node _targetNode, DCapValue _capacity) :
				g(_g), srg_set(_set), affected(*srg_set), connType(_connType), connId(_connId), sourceNode(_sourceNode),
				targetNode(_targetNode), capacity(_capacity), solution_edge(*g)
	{
		ResetDemand();
	};


template <class GR> 	TrafficDemand<GR>::TrafficDemand(const TrafficDemand& _other) :
				g(_other.g), srg_set(_other.srg_set), affected(*srg_set), connType(_other.connType), connId(_other.connId),
				sourceNode(_other.sourceNode), targetNode(_other.targetNode), capacity(
				_other.capacity), solution_edge(*g)
	{
		ResetDemand();
		used_edges.insert(_other.used_edges.begin(), _other.used_edges.end());
		for (ArcIt eit(*g); eit != INVALID; ++eit)
			solution_edge[eit] = _other.solution_edge[eit];

		for (SRLGIt it(*srg_set); it != INVALID; ++it)
			affected[it] = _other.affected[it];
	};


template <class GR> 	const TrafficDemand<GR>& TrafficDemand<GR>::operator=(const TrafficDemand& _other)
	{
		connType = _other.connType;
		connId = _other.connId;
		sourceNode = _other.sourceNode;
		targetNode = _other.targetNode;
		capacity = _other.capacity;
		g = _other.g;
		for (ArcIt eit(*g); eit != INVALID; ++eit)
			solution_edge[eit] = _other.solution_edge[eit];

		for (SRLGIt it(*srg_set); it != INVALID; ++it)
			affected[it] = _other.affected[it];

		return *this;
	};

	class MWLDSimulator{
	public:
		Digraph &g; 
		int routing_method; 

       		int threenines; 
       		int fournines;  
		bool smart_smpair;
		DCapMap cap; 
		DCapMap reduced_cap; 
	   	DCapMap rfd_cap;
 		DCostMap cost; 
		DCostMap delaymin; 
		DCostMap delaymax; 
	  	DCostMap minusdelaymax;
		DDoubleMap id; 
		DFreeCap free_cap; 
		DCoordMap coords; 
		ArcUnavMap link_unav; 
		DNodeUnavMap node_unav; 

		SRLGSet srg_set; 
		DoubleSRLGMap state_prob; 
		IntSRLGMap srg_id; 

		int edge_number; 
		int node_number; 
		int srg_number; 
		int demand_number;
		int bottlenecks; 
		int defi_edges; 
		long double sum_availability; 
		long double min_availability; 
		queue<TrafficDemand<Digraph> > traffic_queue; 
        vector < pair <int, DNodeIt> > sm_used_vector;
        int counter;
        double nullcap;
        double halfcap;
        int seedd;

		double used_edge_number; 
		double used_capacity; 
		int blocked_connections; 
		int ss_blocked_connections; 
		int avail_blocked; 
		int wp_blocked; 
		int pp_blocked; 


		void setEdgesWithEnoughCapacity(DBoolMap& can_be_work_edge, double mincapacity) const; 
	
		void viewSRLGSet(SRLGSet& _srg_set) const;

		MWLDSimulator(Digraph &_g);
		void reset();

		Digraph& getG();
	
		Arc oppositeArc(Arc e){
			for (ArcIt eit(g); eit != INVALID; ++eit)
				if(g.source(eit) == g.target(e) && g.target(eit) == g.source(e)) return eit;
		}

		Arc oppositeArc_new_graph(Arc e){
			for (ArcIt eit(g); eit != INVALID; ++eit)
				if(g.source(eit) == g.target(e) && g.target(eit) == g.source(e) && free_cap[eit]>0) return eit;
		}

		void addToBuiltConnections(TrafficDemand<Digraph>& next_demand, list<TrafficDemand<Digraph> > & built_connections);
		virtual void chooseSRLGList(DNode source, DNode target, BoolSRLGMap & havetoprotect);
		virtual DNode getNode(int DNodeId);
 		virtual void readGraph(ifstream &srg_file, int capacity);
		virtual void readSRLGSet(ifstream &srg_file);
		virtual void readTraffic(ifstream &trf_file);
		virtual void run(){};
		virtual ~MWLDSimulator();
	};
#endif

