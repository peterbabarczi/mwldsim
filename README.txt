Minimum-weight link-disjoint path simulator:
- COMPILE: Add paths accordingly in the Makefile to the following libraries and run make:
	-- Boost (http://www.boost.org/)
	-- LEMON with LEMONRouting extension (http://lemon.cs.elte.hu/trac/lemon)
	-- Gurobi optimizer (http://www.gurobi.com)
- RUN: sh simulate_mwld.sh
- INPUT: All necessary input files (toplogy, traffic, failure model) are provided in the net/ folder.
- OUTPUT: Results will appear in the res/ folder in .xml format.
