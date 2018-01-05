/**
 \file restricted_common_nodes.cpp
 \brief Calcultes restricted common nodes.

 \author              Babarczi Peter
 \author              babarczi@tmit.bme.hu
 \date                2015 november
*/

#include "restricted_common_nodes.h"

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <cstdio>
#include <sstream>
#include <string>

#include <lemon_routing/make_names.h>
#include <lemon_routing/srg_set.h>
#include <lemon_routing/per_flow.h>
#include <lemon_routing/inverse_capacity_proportion.h>


#include <lemon/concepts/graph.h>
#include <lemon/adaptors.h>

using namespace lemon;
using namespace std;


// Initialize maps for the MWLD upper-bound algoirthm
int RestrictedCommonNodes::initialize_MWLD_maps(double type_A_percentage, double type_A_allowed_common_percentage){
	node_num = 0;
	for(DNodeIt nit(g);nit != INVALID;++nit) node_num++;

	int max_type_A_nodes = ceil(type_A_percentage * (double)node_num);
	int max_allowed_type_A_nodes = ceil(type_A_allowed_common_percentage * (double)node_num);


	int edge_cnt = 1;
	for(ArcIt ait(g); ait != INVALID; ++ait){
		cost[ait] = -1;
		delay[ait] = 0;
		visible[ait] = 0;
		transformed_edge[ait] = false;
		transformed_visible_edge[ait] = false;

		edge_id[ait] = edge_cnt;
		edge_cnt++;
	}

//	cout << " ----------------- Nodal degrees ---------------- " << endl;
	for(DNodeIt nit(g);nit != INVALID;++nit){
		//cout << make_name(nit,g) << endl;
		node_cost[nit] = 0;
		node_delay[nit] = 1;
		node_visible[nit] = 1;
	
		int node_deg = 0;
		// We assume that if an arc is given, its anti-parallel arc is also given.
		for(Digraph::InArcIt eit(g, nit);eit != INVALID;++eit)
			node_deg++;

//		cout << make_name(nit,g) << " " << node_deg << endl;

		if(node_deg >= 4){
			type_A_nodes++;
			if(max_type_A_nodes > 0){
				type_A_node[nit] = true;
				max_type_A_nodes--;
				if(max_allowed_type_A_nodes > 0){
					type_A_allowed_common_node[nit] = true;
					max_allowed_type_A_nodes--;
				}
				else type_A_allowed_common_node[nit] = false;
			}
			else{ type_A_node[nit] = false; type_A_allowed_common_node[nit] = false;}
		}
		else{ type_A_node[nit] = false; type_A_allowed_common_node[nit] = false;}
	}
//	cout << " ------------------------------- " << endl;

	if(type_A_percentage * (double)node_num > (double)type_A_nodes) return -1;

	// Set edge costs
	for(ArcIt ait(g); ait != INVALID; ++ait){
		if(type_A_node[g.source(ait)] && type_A_node[g.target(ait)]){
			cost[ait] = 1 + BRandom::instance()->GetNextUniformInt(9);
			cost[oppositeArc(ait)] = cost[ait];		
		}
		else if(type_A_node[g.source(ait)] || type_A_node[g.target(ait)]){
			cost[ait] = 10 + BRandom::instance()->GetNextUniformInt(90); 
			cost[oppositeArc(ait)] = cost[ait];	
		}
		else{
			cost[ait] = 100 + BRandom::instance()->GetNextUniformInt(900); 
			cost[oppositeArc(ait)] = cost[ait];	
		}
	}

	maximal_edge_weight = 0;
	for(ArcIt ait(g); ait != INVALID; ++ait){
		if(cost[ait] > maximal_edge_weight) maximal_edge_weight = cost[ait];
	}

	cout << "maximal_edge_weight " <<  maximal_edge_weight << endl;
//	for(ArcIt ait(g); ait != INVALID; ++ait)
//		cout << make_name("-",ait,g) << " " << cost[ait] << endl;

	return 0;
}

// MWLD problem (ICNP upper bound algorithm)
// Stage 1: Create Transformed Graph; Stage 2: Calculate Restricted Shortest Path; Stage 3: Save shortest pairs of link-disjoint paths in the original graph
void RestrictedCommonNodes::calculateMWLD(int max_common_node_num, int method, rusage &MWLDStage1Creation){
	SplitGraph splitnodes(g); 
	CombinedCostMap combined_cost(cost, node_cost);
	CombinedCostMap combined_delay(delay, node_delay);
	CombinedCostMap combined_visible(visible, node_visible); 

	// Stage 1: create transformed graph.
	cout << "\t Stage 1: Create transformed graph G'." << endl;
	createTransformedGraph(splitnodes, combined_cost, combined_delay, combined_visible);

	getrusage(RUSAGE_SELF, &MWLDStage1Creation);

	node_num = 0;
	for(DNodeIt nit(g);nit != INVALID;++nit) node_num++;

	queue<TrafficDemand<Digraph> > temp_traffic_queue(traffic_queue);
	while (!temp_traffic_queue.empty())
	{
		TrafficDemand<Digraph> next_demand = temp_traffic_queue.front();

		temp_traffic_queue.pop();

		// Build a connection.
		if (next_demand.connType == BUILD)
		{
			// Based on the source and target node we have to reset the usable edges in the selective common node algorithm
			// visible[ait] required for the RSP ILP, transformed_visible_edge[ait] is used in the Bellman-Ford variant
			if(method == 31 || method == 32){
				for(ArcIt ait(g); ait != INVALID; ++ait){
					visible[ait] = 0;
					transformed_visible_edge[ait] = false;
				}

				for(ArcIt ait(g); ait != INVALID; ++ait){
					if(transformed_edge[ait]){
						if(type_A_allowed_common_node[g.source(ait)] || g.id(g.source(ait)) == g.id(next_demand.sourceNode)){
							if(type_A_allowed_common_node[g.target(ait)] || g.id(g.target(ait)) == g.id(next_demand.targetNode)){
								visible[ait] = 1;
								transformed_visible_edge[ait] = true;
							}
						}
					}
				}
			}

			cout << "\t Stage 2: Calculate shortest path for " << next_demand.connId << " connection between " << make_name(next_demand.sourceNode,g) << " " << make_name(next_demand.targetNode,g) << " with " << max_common_node_num << " in G'." << endl;

			// Restricted Shortest Path ILP calculating Stage 2
			if(method == 11 || method == 31){
				enum MipSolver::ColTypes col_type = (MipSolver::ColTypes)((((((((1))))))));
				typedef SplitGraph::ArcMap<Mip::Col> LPVariableMap;
				Mip lp;
				LPVariableMap var_x(splitnodes);
				for(SplitGraph::ArcIt e(splitnodes);e != INVALID;++e){
					if(combined_visible[e] > 0){
						var_x[e] = lp.addCol();
						lp.colBounds(var_x[e], 0, 1);
						std::ostringstream name;
						name << make_name("-", e, splitnodes);
						lp.colName(var_x[e], name.str());
						lp.colType(var_x[e], col_type);
					}
				}

				for (SplitGraph::NodeIt n(splitnodes); n != INVALID; ++n){
					Mip::Expr ex;
					for(SplitGraph::InArcIt e(splitnodes, n);e != INVALID;++e)
						if(combined_visible[e] > 0) ex += var_x[e];

					for(SplitGraph::OutArcIt e(splitnodes, n);e != INVALID;++e)
						if(combined_visible[e] > 0) ex -= var_x[e];

					if(n == splitnodes.outNode(next_demand.sourceNode))
						lp.addRow(ex == -1);

					else{
						if(n == splitnodes.inNode(next_demand.targetNode))
							lp.addRow(ex == 1);

						else
							lp.addRow(ex == 0);
					}
				}

				Mip::Expr del_bound;
				for(SplitGraph::ArcIt e(splitnodes);e != INVALID;++e)
					if(combined_visible[e] > 0) del_bound += combined_delay[e]* var_x[e];
				
				lp.addRow(del_bound <= max_common_node_num);

				Mip::Expr obj;
				for(SplitGraph::ArcIt e(splitnodes);e != INVALID;++e)
					if(combined_visible[e] > 0) obj += combined_cost[e] * var_x[e];

				lp.obj(obj);
				lp.min();
				lp.solve();

				next_demand.used_edges.clear();
				bool blocked = false;

				if(lp.type() != lemon::MipSolver::OPTIMAL && lp.type() != lemon::MipSolver::FEASIBLE){	
					cout << "\t No somehow-disjoint paths exist with given parameters between " << g.id(next_demand.sourceNode) << " and " << g.id(next_demand.targetNode) << endl;
					if(next_demand.connId >= (demand_number/2))
						ss_blocked_connections++;

					blocked_connections++;
					wp_blocked++;
					blocked = true;
					//return;
				}
				else{	
					// Stage 3: Map back the shortest path in the transformed graph to the disjoint path-pairs in the original graph.
					cout << "\t Stage 3: Map back RSP shortest path to original links: ";
					DNodeIntMap path_degree(g,0);
					int common_nodes = 0;
					int common_nodes_after_mapping_back = 0;
					for(SplitGraph::ArcIt pe(splitnodes); pe != INVALID; ++pe){
						if(combined_visible[pe] > 0){
							if(lp.sol(var_x[pe]) >= 0.0001){
								//cout << make_name("-", pe, splitnodes) << " " << lp.sol(var_x[pe])*next_demand.capacity << endl;
								if(splitnodes.bindArc(pe)){
									common_nodes++;
								}
								else{
									//cout << "\t Original edges for " << make_name("-", pe, splitnodes) << ": ";
									for(set<int>::iterator oeit = original_edges[pe].begin(); oeit != original_edges[pe].end();++oeit){ 
										for(ArcIt ait(g); ait != INVALID; ++ait){
											if(edge_id[ait] == *oeit && !next_demand.solution_edge[ait]){
												total_solcost += ((double)lp.sol(var_x[pe])*(double)cost[ait]);
												next_demand.used_edges.insert(make_pair(make_name("-", ait, g), lp.sol(var_x[pe])));
												cout << make_name("-",ait,g) << " ";
												next_demand.solution_edge[ait] = true;	
												//path_degree[g.source(ait)]++;
												path_degree[g.target(ait)]++;
											}
										}
									}
									//cout << endl;
								}
							}
						}
					}
					cout << endl;

					if(!blocked) addToBuiltConnections(next_demand, built_connections);

					// Save statistics for output file.
					Suurballe<Digraph, DCostMap> link_disjoint_suurballe(g, cost);
					link_disjoint_suurballe.fullInit(next_demand.sourceNode);
					link_disjoint_suurballe.start(next_demand.targetNode, 2);

					Suurballe<SplitGraph, CombinedCostMap> node_disjoint_suurballe(splitnodes, combined_cost);
					node_disjoint_suurballe.fullInit(splitnodes.outNode(next_demand.sourceNode));
					node_disjoint_suurballe.start(splitnodes.inNode(next_demand.targetNode), 2);
					
					link_disoint_cost += link_disjoint_suurballe.totalLength();
					node_disoint_cost += node_disjoint_suurballe.totalLength();

					// Number of connections requests with possible improvement.
					if(node_disjoint_suurballe.totalLength() > link_disjoint_suurballe.totalLength())
						improvable_connections++;

					if(node_disjoint_suurballe.totalLength() > lp.sol(obj)){
						connections_with_benefits++;
						mwld_improvement_to_node_disjoint += (node_disjoint_suurballe.totalLength() - lp.sol(obj));
						if(link_disjoint_suurballe.totalLength()  == lp.sol(obj))
							allowing_common_nodes_leads_to_optimum++;
					}

					// Number of optimal solutions.
					if(link_disjoint_suurballe.totalLength() == lp.sol(obj)){
						min_cost_connections++;
					}

					link_disjoint_improvement_to_node_disjoint += (node_disjoint_suurballe.totalLength() - link_disjoint_suurballe.totalLength());
					
					//cout << "\t Common nodes: " << common_nodes << endl; 
					if(common_nodes < min_common_nodes)	min_common_nodes = common_nodes;
					if(common_nodes > max_common_nodes)	max_common_nodes = common_nodes;
					avg_common_nodes += (double)common_nodes;

					// Check if the solutin is correct or not! We count only the in-degrees!
					cout << "\t\t Common node(s) in the paths: ";
					for(DNodeIt nit(g); nit != INVALID; ++nit){
						if(path_degree[nit] == 2 && !(nit == next_demand.targetNode)){
							cout << make_name(nit,g) << " " ;
							common_nodes_after_mapping_back++;
						}
						else if(path_degree[nit] > 2){
							cout << "\t\t ERROR: More than two paths? " << endl;
							return;
						}
					}
					cout << endl;

					
					if((common_nodes > max_common_node_num) || (common_nodes_after_mapping_back  > common_nodes)){
						cout << "\t\t ERROR: More common nodes than allowed!" << endl;
						return;
					}
				}
			}



			// Bellman-Ford implementation of Stage 2
			else if(method == 12 || method == 32){
				DFilteredGraph transformed_graph(g, transformed_visible_edge);

				// [i][v]: shortest path with at most i links to node v
				int bellman_ford_distance[node_num][node_num];
				int bellman_ford_predecessor[node_num][node_num];

				// Initialization
				//cout << "INT_MAX: " << INT_MAX << endl;
				for(int i = 0; i < node_num; ++i){
					for(int v = 0; v < node_num; ++v){
						if(i == 0 && v == g.id(next_demand.sourceNode)) bellman_ford_distance[i][v] = 0;
						else bellman_ford_distance[i][v] = INT_MAX - maximal_edge_weight;
					}
				}
				for(int i = 0; i < node_num; ++i){
					for(int v = 0; v < node_num; ++v){
						bellman_ford_predecessor[i][v] = -1;
					}
				}
				
				// Run the relaxation
				for(int i = 1; i <= max_common_node_num + 1; ++i){
					for(DFilteredGraph::ArcIt feit(transformed_graph); feit != INVALID; ++feit){
						if(bellman_ford_distance[i-1][g.id(g.target(feit))] < bellman_ford_distance[i][g.id(g.target(feit))]){
							bellman_ford_distance[i][g.id(g.target(feit))] = bellman_ford_distance[i-1][g.id(g.target(feit))];
							bellman_ford_predecessor[i][g.id(g.target(feit))] = bellman_ford_predecessor[i-1][g.id(g.target(feit))];
						}
						// Difference in relaxation: <= is used, i.e., form equal cost path the one with maximal links will be found
						if((bellman_ford_distance[i-1][g.id(g.source(feit))] + cost[feit]) <= bellman_ford_distance[i][g.id(g.target(feit))]){
							bellman_ford_distance[i][g.id(g.target(feit))] = (bellman_ford_distance[i-1][g.id(g.source(feit))] + cost[feit]);
							bellman_ford_predecessor[i][g.id(g.target(feit))] = edge_id[feit];
						}
					}

					// View after ith iteration
					/*cout << i << "th iteration:";
					for(int v = 0; v < node_num; ++v){
						if(bellman_ford_distance[i][v] != INT_MAX - maximal_edge_weight) cout << bellman_ford_distance[i][v] << " ";
						else cout << "inf ";
					}
					cout << endl;
					cout << i << "th iteration:";
					for(int v = 0; v < node_num; ++v){
						cout << bellman_ford_predecessor[i][v] << " ";
					}
					cout << endl;*/
				}
				
				next_demand.used_edges.clear();
				bool blocked = false;

				if(bellman_ford_distance[max_common_node_num + 1][g.id(next_demand.targetNode)] == INT_MAX - maximal_edge_weight){	
					cout << "\t No somehow-disjoint paths exist with given parameters between " << g.id(next_demand.sourceNode) << " and " << g.id(next_demand.targetNode) << endl;
					if(next_demand.connId >= (demand_number/2))
						ss_blocked_connections++;

					blocked_connections++;
					wp_blocked++;
					blocked = true;
					//return;
				}
				else{		
					// Stage 3: Map back the shortest path in the transformed graph to the disjoint path-pairs in the original graph.
					cout << "\t Stage 3: Map back BF shortest path to original links: ";
					DNodeIntMap path_degree(g,0);
					int next_predecessor_iteration = max_common_node_num + 1;
					int common_nodes = 0;
					int common_nodes_after_mapping_back = 0;
					int solutoin_cost_connection = 0;

					DNode predecessor = next_demand.targetNode;
					while(g.id(predecessor) != g.id(next_demand.sourceNode)){
						for(DFilteredGraph::ArcIt feit(transformed_graph); feit != INVALID; ++feit){
							if(edge_id[feit] == bellman_ford_predecessor[next_predecessor_iteration][g.id(predecessor)]){
								//cout << "\t Original edges for " << make_name("-", feit, g) << ": ";
								for(set<int>::iterator oeit = original_edges[feit].begin(); oeit != original_edges[feit].end();++oeit){ 
									for(ArcIt ait(g); ait != INVALID; ++ait){
										if(edge_id[ait] == *oeit && !next_demand.solution_edge[ait]){
											//cout << "Current: " << bellman_ford_predecessor[next_predecessor_iteration][g.id(predecessor)] << " " << g.id(predecessor) << endl;
											
											path_degree[g.target(ait)]++;
											next_demand.solution_edge[ait] = true;
											next_demand.used_edges.insert(make_pair(make_name("-", ait, g), 1.0));
											cout << make_name("-", ait, g) << " ";

											total_solcost += cost[ait];
											solutoin_cost_connection += cost[ait];
										}
									}
								}
							
								// Set next iteration.
								predecessor = g.source(feit);
								next_predecessor_iteration--;
								if(g.id(predecessor) != g.id(next_demand.sourceNode)) common_nodes++;
							}
						}
					}
					cout << endl;

					if(!blocked) addToBuiltConnections(next_demand, built_connections);

					// Save statistics for output file.
					Suurballe<Digraph, DCostMap> link_disjoint_suurballe(g, cost);
					link_disjoint_suurballe.fullInit(next_demand.sourceNode);
					link_disjoint_suurballe.start(next_demand.targetNode, 2);

					Suurballe<SplitGraph, CombinedCostMap> node_disjoint_suurballe(splitnodes, combined_cost);
					node_disjoint_suurballe.fullInit(splitnodes.outNode(next_demand.sourceNode));
					node_disjoint_suurballe.start(splitnodes.inNode(next_demand.targetNode), 2);
					
					link_disoint_cost += link_disjoint_suurballe.totalLength();
					node_disoint_cost += node_disjoint_suurballe.totalLength();

					// Number of connections requests with possible improvement.
					if(node_disjoint_suurballe.totalLength() > link_disjoint_suurballe.totalLength())
						improvable_connections++;

					if(node_disjoint_suurballe.totalLength() > solutoin_cost_connection){
						connections_with_benefits++;
						mwld_improvement_to_node_disjoint += (node_disjoint_suurballe.totalLength() - solutoin_cost_connection);
						if(link_disjoint_suurballe.totalLength()  == solutoin_cost_connection)
							allowing_common_nodes_leads_to_optimum++;
					}

					// Number of optimal solutions.
					if(link_disjoint_suurballe.totalLength() == solutoin_cost_connection){
						min_cost_connections++;
					}

					link_disjoint_improvement_to_node_disjoint += (node_disjoint_suurballe.totalLength() - link_disjoint_suurballe.totalLength());
					
					//cout << "\t Common nodes: " << common_nodes << endl; 
					if(common_nodes < min_common_nodes)	min_common_nodes = common_nodes;
					if(common_nodes > max_common_nodes)	max_common_nodes = common_nodes;
					avg_common_nodes += (double)common_nodes;

					// Check if the solutin is correct or not! We count only the in-degrees!
					cout << "\t\t Common node(s) in the paths: ";
					for(DNodeIt nit(g); nit != INVALID; ++nit){
						if(path_degree[nit] == 2 && !(nit == next_demand.targetNode)){
							cout << make_name(nit,g) << " " ;
							common_nodes_after_mapping_back++;
						}
						else if(path_degree[nit] > 2){
							cout << "\t\t ERROR: More than two paths? " << endl;
							return;
						}
					}
					cout << endl;

					
					if((common_nodes > max_common_node_num) || (common_nodes_after_mapping_back  > common_nodes)){
						cout << "\t\t ERROR: More common nodes than allowed!" << endl;
						return;
					}
				}
			}
		}
	}
}

// MWLD (ICNP) Stage 1: Create transformed graph with the same node set and with edges representing node-disjoint path-pairs.
void RestrictedCommonNodes::createTransformedGraph(SplitGraph &splitnodes, CombinedCostMap &combined_cost, CombinedCostMap &combined_delay, CombinedCostMap &combined_visible){
	int next_edge_id = 0;
	for(ArcIt eit(g); eit != INVALID; ++eit) next_edge_id++;

	Suurballe<SplitGraph, CombinedCostMap> suurballe(splitnodes, combined_cost);
	for(DNodeIt nit(g); nit != INVALID; ++nit){
		suurballe.fullInit(splitnodes.outNode(nit));
		for(DNodeIt bnit(g);bnit != INVALID; ++bnit){
			if(nit != bnit){
				suurballe.start(splitnodes.inNode(bnit), 2);
				if (suurballe.pathNum() >= 2){
					Arc e = g.addArc(nit, bnit);
					cost[e] = suurballe.totalLength();
					delay[e] = 0;
					visible[e] = 1;
					transformed_edge[e] = true;
					transformed_visible_edge[e] = true;

					next_edge_id++;
					edge_id[e] = next_edge_id;

					Path<SplitGraph> primary_path = suurballe.path(0);
					Path<SplitGraph> secondary_path = suurballe.path(1);

					for(Path<SplitGraph>::ArcIt pe(primary_path);pe != INVALID;++pe){ 
						if(!splitnodes.bindArc(pe)){
							for (ArcIt eit(g); eit != INVALID; ++eit){
								if(splitnodes.arc(eit) == pe){
									original_edges[e].insert(edge_id[eit]);
								}
							}
						}
					}
					for(Path<SplitGraph>::ArcIt pe(secondary_path);pe != INVALID;++pe){
						if(!splitnodes.bindArc(pe)){
							for (ArcIt eit(g); eit != INVALID; ++eit){
								if(splitnodes.arc(eit) == pe){
									original_edges[e].insert(edge_id[eit]);
								}
							}
						}
					}
					//cout << "Size of new edge " << make_name("-",e,g) << " (" << make_name("-",splitnodes.arc(e),splitnodes) << ") is " << original_edges[e].size() <<endl;
				}
			}
		}
	}
}

// Restrcited Conditional Second Shortest Path (ToN) Stage 1: Calculate shortest path, remove its links; Stage 2: Run modified Bellmann Ford
void RestrictedCommonNodes::calculateConditionalSecondPath(int max_common_node_num, int method){
	queue<TrafficDemand<Digraph> > temp_traffic_queue(traffic_queue);
	while (!temp_traffic_queue.empty())
	{
		// Re-initialize maps
		for(DNodeIt nit(g); nit != INVALID; ++nit) on_working[nit] = false;
		for(ArcIt eit(g); eit != INVALID; ++eit) non_working_edge[eit] = true;
		int internal_working_path_nodes = 0;

		TrafficDemand<Digraph> next_demand = temp_traffic_queue.front();

		temp_traffic_queue.pop();

		// Build a connection.
		if (next_demand.connType == BUILD)
		{
			int solutoin_cost_connection = 0;
			bool blocked = false;
			cout << "\t Stage 1: Calculate shortest path in G: "; // << endl;
			Dijkstra<Digraph, DCostMap> dijkstra(g, cost);
			DPath working_path;

			if(!dijkstra.run(next_demand.sourceNode, next_demand.targetNode)){
				cout << "Dijkstra failed from " << g.id(next_demand.sourceNode) << " to " << g.id(next_demand.targetNode) << endl;
				if(next_demand.connId >= (demand_number/2))
					ss_blocked_connections++;

				blocked_connections++;
				wp_blocked++;
				blocked = true;
			}
			
			if(!blocked){
				working_path = dijkstra.path(next_demand.targetNode);

				//cout << "\t Working path: ";
				for(DPath::ArcIt eit(working_path); eit != INVALID; ++eit){
					non_working_edge[eit] = false;
					// If we only remove a link in one direction, the second path may traverse it in the opposite direction.
					//non_working_edge[oppositeArc(eit)] = false;

					cout << make_name("-", eit, g) << " ";

					if(make_name(g.target(eit),g) != make_name(next_demand.targetNode,g)){
						on_working[g.target(eit)] = true;
						internal_working_path_nodes++;
					}
				}
				cout << endl;
				
				// Remove shortest path links from the graph.
				DFilteredGraph enough_capacity_graph(g, non_working_edge);

				cout << "\t Stage 2: Calculate restricted conditional second shortest path for " << next_demand.connId << " connection between " << make_name(next_demand.sourceNode,g) << " " << make_name(next_demand.targetNode,g) << " with " << max_common_node_num << " in G'." << endl;

				// [i][v][c]: shortest path with at most i links to node v with c common nodes
				int bellman_ford_distance[node_num][node_num][internal_working_path_nodes + 1];
				int bellman_ford_predecessor[node_num][internal_working_path_nodes + 1];

				// Initialization
				for(int i = 0; i < node_num; ++i){
					for(int v = 0; v < node_num; ++v){
						for(int c = 0; c < internal_working_path_nodes + 1; ++c){
							if(i == 0 && v == g.id(next_demand.sourceNode) && c == 0) bellman_ford_distance[i][v][c] = 0;
							else bellman_ford_distance[i][v][c] = INT_MAX - maximal_edge_weight;
						}
					}
				}
				for(int v = 0; v < node_num; ++v){
					for(int c = 0; c < internal_working_path_nodes + 1; ++c){
						bellman_ford_predecessor[v][c] = -1;
					}
				}

				// Run the relaxation
				for(int i = 1; i < node_num; ++i){
					for(DFilteredGraph::ArcIt feit(enough_capacity_graph); feit != INVALID; ++feit){
						if(on_working[g.target(feit)]){
							for(int c = 1; c < internal_working_path_nodes + 1; ++c){
								if(bellman_ford_distance[i-1][g.id(g.target(feit))][c] < bellman_ford_distance[i][g.id(g.target(feit))][c])
									bellman_ford_distance[i][g.id(g.target(feit))][c] = bellman_ford_distance[i-1][g.id(g.target(feit))][c];
								if((bellman_ford_distance[i-1][g.id(g.source(feit))][c-1] + cost[feit]) < bellman_ford_distance[i][g.id(g.target(feit))][c]){
									bellman_ford_distance[i][g.id(g.target(feit))][c] = (bellman_ford_distance[i-1][g.id(g.source(feit))][c-1] + cost[feit]);
									bellman_ford_predecessor[g.id(g.target(feit))][c] = edge_id[feit];
								}
							}
						}
						else{
							for(int c = 0; c < internal_working_path_nodes + 1; ++c){
								if(bellman_ford_distance[i-1][g.id(g.target(feit))][c] < bellman_ford_distance[i][g.id(g.target(feit))][c])
									bellman_ford_distance[i][g.id(g.target(feit))][c] = bellman_ford_distance[i-1][g.id(g.target(feit))][c];
								if((bellman_ford_distance[i-1][g.id(g.source(feit))][c] + cost[feit]) < bellman_ford_distance[i][g.id(g.target(feit))][c]){
									bellman_ford_distance[i][g.id(g.target(feit))][c] = (bellman_ford_distance[i-1][g.id(g.source(feit))][c] + cost[feit]);
									bellman_ford_predecessor[g.id(g.target(feit))][c] = edge_id[feit];
								}
							}
						}
					}

				}

				// Get the second shortest path with the required number of common nodes
				int best_common_node_num = INT_MAX;
				int best_cost = INT_MAX - maximal_edge_weight;
				
				// Upper bound from range [0,k]
				if(method == 21){
					for(int c = 0; (c <= max_common_node_num) && (c < internal_working_path_nodes + 1); ++c){
						if(bellman_ford_distance[node_num-1][g.id(next_demand.targetNode)][c] < best_cost){
							best_common_node_num = c;
							best_cost = bellman_ford_distance[node_num-1][g.id(next_demand.targetNode)][c];
						}
					}
				}
				// Tight bound [k]
				else if(method == 22){
					if(max_common_node_num <= internal_working_path_nodes){
						best_common_node_num = max_common_node_num;
						best_cost = bellman_ford_distance[node_num-1][g.id(next_demand.targetNode)][max_common_node_num];
					}
				}
				// Lower bound from range [k, l_1]
				else if (method == 23){
					if(max_common_node_num <= internal_working_path_nodes){
						for(int c = max_common_node_num; c < internal_working_path_nodes + 1; ++c){
							if(bellman_ford_distance[node_num-1][g.id(next_demand.targetNode)][c] < best_cost){
								best_common_node_num = c;
								best_cost = bellman_ford_distance[node_num-1][g.id(next_demand.targetNode)][c];
							}
						}
					}
				}

				if(best_cost < INT_MAX - maximal_edge_weight) cout << "\t\t Second shortest path has cost " << best_cost << " with " << best_common_node_num << " common node(s): "; 
				else{
					cout << "\t\t Second path search failed from " << g.id(next_demand.sourceNode) << " to " << g.id(next_demand.targetNode) << endl;
					if(next_demand.connId >= (demand_number/2))
						ss_blocked_connections++;

					blocked_connections++;
					blocked = true;
				}

				if(!blocked){
					// Add the working path edges to the solution.
					for(DPath::ArcIt eit(working_path); eit != INVALID; ++eit){						
						next_demand.solution_edge[eit] = true;
						next_demand.used_edges.insert(make_pair(make_name("-", eit, g), 1.0));

						total_solcost += cost[eit];
						solutoin_cost_connection += cost[eit];

						used_edge_number++;
						used_capacity += next_demand.capacity;
						avg_wp_length++;
					}

					// Get the second path recursively.
					DNode predecessor = next_demand.targetNode;
					int next_common_node_num = best_common_node_num;
					while(g.id(predecessor) != g.id(next_demand.sourceNode)){
						for(DFilteredGraph::ArcIt feit(enough_capacity_graph); feit != INVALID; ++feit){
							if(edge_id[feit] == bellman_ford_predecessor[g.id(predecessor)][next_common_node_num]){
								//cout << "Current: " << bellman_ford_predecessor[g.id(predecessor)][next_common_node_num] << " " << g.id(predecessor) << " " << next_common_node_num << endl;

								next_demand.solution_edge[feit] = true;
								next_demand.used_edges.insert(make_pair(make_name("-", feit, g), 1.0));
								cout << make_name("-", feit, g) << " ";

								total_solcost += cost[feit];
								solutoin_cost_connection += cost[feit];

								used_edge_number++;
								used_capacity += next_demand.capacity;

								// Set next iteration.
								if(on_working[g.target(feit)]){
									predecessor = g.source(feit);
									next_common_node_num--;
								}
								else{
									predecessor = g.source(feit);
								}
								//cout << "New: " << bellman_ford_predecessor[g.id(predecessor)][next_common_node_num] << " " << g.id(predecessor) << " " << next_common_node_num << endl;
							}
						}
					}
					cout << endl;
					

					if(!blocked) addToBuiltConnections(next_demand, built_connections);

					// Save statistics for output file.
					Suurballe<Digraph, DCostMap> link_disjoint_suurballe(g, cost);
					link_disjoint_suurballe.fullInit(next_demand.sourceNode);
					link_disjoint_suurballe.start(next_demand.targetNode, 2);

					SplitGraph splitnodes(g); 
					CombinedCostMap combined_cost(cost, node_cost);
					Suurballe<SplitGraph, CombinedCostMap> node_disjoint_suurballe(splitnodes, combined_cost);
					node_disjoint_suurballe.fullInit(splitnodes.outNode(next_demand.sourceNode));
					node_disjoint_suurballe.start(splitnodes.inNode(next_demand.targetNode), 2);
					
					link_disoint_cost += link_disjoint_suurballe.totalLength();
					node_disoint_cost += node_disjoint_suurballe.totalLength(); 

					// Number of connections requests with possible improvement.
					if(node_disjoint_suurballe.totalLength() > link_disjoint_suurballe.totalLength())
						improvable_connections++;

					if(node_disjoint_suurballe.totalLength() > solutoin_cost_connection){
						connections_with_benefits++;
						mwld_improvement_to_node_disjoint += (node_disjoint_suurballe.totalLength() - solutoin_cost_connection);
						if(link_disjoint_suurballe.totalLength()  == solutoin_cost_connection)
							allowing_common_nodes_leads_to_optimum++;
					}

					// Number of optimal solutions.
					if(link_disjoint_suurballe.totalLength() == solutoin_cost_connection){
						min_cost_connections++;
					}

					link_disjoint_improvement_to_node_disjoint += (node_disjoint_suurballe.totalLength() - link_disjoint_suurballe.totalLength());
					
					//cout << "\t Common nodes: " << best_common_node_num << endl; 
					if(best_common_node_num < min_common_nodes)	min_common_nodes = best_common_node_num;
					if(best_common_node_num > max_common_nodes)	max_common_nodes = best_common_node_num;
					avg_common_nodes += (double)best_common_node_num;
				}
			}
		}
	}
}
