/**
 \file mwld_simulator.cpp

 \author              Babarczi Peter
 \author              babarczi@tmit.bme.hu
 \date                2015 november
*/

#include "mwld_simulator.h"

#include <sstream>

    void MWLDSimulator::reset()
	{
		threenines = 0;
		fournines = 0;
		sum_availability = 0;
		min_availability = 1.0;
		avail_blocked = 0;
		wp_blocked = 0;
		pp_blocked = 0;
		used_edge_number = 0;
        used_capacity = 0;
		blocked_connections = 0;
        ss_blocked_connections = 0;
        counter = 0;
        smart_smpair = false;
        nullcap = 0;
        halfcap = 0;
        seedd = 1;
	}

    MWLDSimulator::MWLDSimulator(Digraph &_g) :
		g(_g), cap(g),reduced_cap(g),rfd_cap(g), id(g),
		cost(g), delaymin(g), delaymax(g), minusdelaymax(g), free_cap(g), coords(g), srg_set(g), state_prob(srg_set), srg_id(srg_set), link_unav(g), node_unav(g),
	  	avail_blocked(0), wp_blocked(0), pp_blocked(0), threenines(0), fournines(0) // deficit_proc(0), extra_proc(0), deficit_num(0), extra_num(0), deficit_p(false), extra_p(false)
	{
		edge_number = 0;
		node_number = 0;
		srg_number = 0;
        demand_number = 0;
		bottlenecks = 0;
		sum_availability = 0.0;
		min_availability = 1.0;
		used_edge_number = 0;
        used_capacity = 0;
		blocked_connections = 0;
		ss_blocked_connections = 0;
		counter = 0;
        smart_smpair = false;
        nullcap = 0;
        halfcap = 0;
        seedd = 1;
	}

	Digraph& MWLDSimulator::getG()
	{
		return g;
	}

	DNode MWLDSimulator::getNode(int nodeId)
	{
		for (DNodeIt nit(g); nit != INVALID; ++nit)
			if (g.id(nit) == nodeId)
			{
				return nit;
			}
	}

	void MWLDSimulator::setEdgesWithEnoughCapacity(DBoolMap& can_be_work_edge, double mincapacity) const
	{
		for (ArcIt eit(g); eit != INVALID; ++eit)
		{
			if (mincapacity > free_cap[eit])
				can_be_work_edge[eit] = false;
		}
	}

    void MWLDSimulator::chooseSRLGList(DNode source, DNode target, BoolSRLGMap & havetoprotect)
    {
        int num = 0;
        for(SRLGIt srg_it(srg_set);srg_it != INVALID;++srg_it){
            havetoprotect[srg_it] = false;
        }
        for(SRLGIt srg(srg_set);srg != INVALID;++srg){
            DBoolMap should_be_use(g);
            for(ArcIt it(g);it != INVALID;++it)
                should_be_use[it] = true;

            for(SRLGArcIt srg_edge_it(srg_set, srg);srg_edge_it != INVALID;++srg_edge_it)
                should_be_use[srg_edge_it] = false;

            DFilteredGraph allowed_subgraph(g, should_be_use);
            DFDijkstra prot_dijkstra(allowed_subgraph, cost);
            if(prot_dijkstra.run(source, target)){
                havetoprotect[srg] = true;
                num++;
            }
        }

        //cout << "SRLGs need protection: " << num << endl;
    }


	void MWLDSimulator::addToBuiltConnections(TrafficDemand<Digraph>& next_demand, list<TrafficDemand<Digraph> > & built_connections)
    {
        for(ArcIt eit(g);eit != INVALID;++eit){
            for(map<string,double>::iterator str_it = next_demand.used_edges.begin();str_it != next_demand.used_edges.end();++str_it){
                string compare = str_it->first;
                double percentage = str_it->second;
                if(make_name("-", eit, g) == compare){
                    free_cap[eit] = free_cap[eit] - next_demand.capacity * percentage;
					free_cap[oppositeArc(eit)] = free_cap[oppositeArc(eit)] - next_demand.capacity * percentage;
                    used_edge_number = used_edge_number + percentage;
                    used_capacity = used_capacity + next_demand.capacity * percentage;
                }
            }

        }
        built_connections.push_back(next_demand);
    }


	//! Read input graph
	void MWLDSimulator::readGraph(ifstream &srg_file, int capacity)
	{
		//cout << "Reading the graph from input file..." <<endl;
		DigraphReader<Digraph> unavreader(g, srg_file);
        unavreader.nodeMap("coords", coords) .nodeMap("unav", node_unav) .arcMap("unav", link_unav).run();

		for (ArcIt eit(g); eit != INVALID; ++eit)
			edge_number++;
		edge_number /= 2; // As the graph is directed
		for (DNodeIt nit(g); nit != INVALID; ++nit)
			node_number++;

		for (ArcIt eit(g); eit != INVALID; ++eit){
			cap[eit] = capacity;
			free_cap[eit] = cap[eit];
			cost[eit] = 1;
			rnd = random();
			int delayconst = 5;
            DNode n1 = g.source(eit);
            DNode n2 = g.target(eit);
            lemon::dim2::Point<float> tempinput = coords[n2] - coords[n1];
            delaymin[eit] = sqrt(tempinput.normSquare())*delayconst;
            delaymax[eit] = sqrt(tempinput.normSquare())*delayconst;
            minusdelaymax[eit] = -1*delaymax[eit];
		}
	}

	//! Read connection requests.
	void MWLDSimulator::readTraffic(ifstream &trf_file)
	{
		//cout << endl << "Reading the traffic file..." <<endl;
		while (!trf_file.eof())
		{
			string mode;
			int id;
			trf_file >> mode;
			if (mode == "c")
			{ // create connection
				demand_number++;
				int connId;
				int sourceNodeId;
				int targetNodeId;
				DCapValue capacity;

				trf_file >> connId;
				trf_file >> sourceNodeId;
				trf_file >> targetNodeId;
				trf_file >> capacity;
				traffic_queue.push(TrafficDemand<Digraph>(&g, &srg_set, BUILD, connId, getNode(sourceNodeId),
						getNode(targetNodeId), capacity));
			}
			else if (mode == "r")
			{
				int connId;
				trf_file >> connId;
				traffic_queue.push(TrafficDemand<Digraph>(&g, &srg_set, RELEASE, connId, DNode(), DNode(), 0));
			}
			trf_file.ignore(256, '\n');
		}
		cout << "trfqueue size: " << traffic_queue.size() << endl;


	}

	//! Read failure list, contains single link failures by default.
    void MWLDSimulator::readSRLGSet(ifstream &srg_file)
	{
		log_msg(2) << "Reading the SRLGSet from input file..." <<endl;
	queue<int> id_q;
	queue<vector<string> > edges_q;
	queue<long double> state_prob_q;
	SRLGReader srg_reader;
	srg_reader.readSRGSet(srg_file, id_q, edges_q, state_prob_q);
	for(int i = id_q.size(); i > 0; i--){
		SRLG srg = srg_set.addSRG();
		log_msg(2) << "\nSRLG "<<make_name(srg,srg_set)<<" ";
		srg_id[srg] = id_q.front();
		state_prob[srg] = state_prob_q.front();
		for(vector<string>::iterator string_it = edges_q.front().begin(); string_it != edges_q.front().end(); string_it++){
			for(ArcIt eit(g); eit != INVALID; ++eit){
				if( make_name("-", eit, g) == *string_it ){
					srg_set.addArcToSRG(srg,eit);
					log_msg(2) << "	" << make_name("-", eit, g);
					break;
				}
			}
		}
		id_q.pop();
		edges_q.pop();
		state_prob_q.pop();
	}

		for(SRLGIt srg_it(srg_set);srg_it != INVALID;++srg_it) srg_number++;

	}


	void MWLDSimulator::viewSRLGSet(SRLGSet& _srg_set) const
	{
		//cout<<endl<<"Now we walk throught the SRLG-s and the arcs in them."<<endl;
		for (SRLGIt srg_it(_srg_set); srg_it != INVALID; ++srg_it)
		{
			cout << "SRLG: " << make_name(srg_it, _srg_set) << endl;
			cout << " and the arcs in it:" << endl;
			for (SRLGArcIt srg_edge_it(_srg_set, srg_it); srg_edge_it != INVALID; ++srg_edge_it)
			{
				cout << "        arc: " << make_name("->", srg_edge_it, g) << endl;
			}
		}
	}

	MWLDSimulator::~MWLDSimulator()
	{
	}

