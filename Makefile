# Project: mwldsim
###########################################################
mwldsim:	GUROBIDIR=/opt/Gurobi/gurobi604/linux64
mwldsim:	LEMONDIR = -I ../lemon-1.3 
mwldsim:	BOOSTDIR = -I ../tools-2.1/common_includes
mwldsim:	SYSTEM     = x86-64_debian4.0_4.1
mwldsim:	LIBFORMAT  = static_pic
mwldsim:	GUROBILIBDIR= $(GUROBIDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
mwldsim:	CLNFLAGS   = -L$(GUROBIDIR)/lib -lgurobi_c++ -L$(GUROBIDIR)/lib -lgurobi60 -lm -lpthread
mwldsim:	CFLAGS_=-I $(GUROBIDIR)/include -g
mwldsim:	all
mwldsim:	CPP  = g++ -D__DEBUG__
mwldsim:	CC   = gcc -D__DEBUG__
###########################################################


OBJ0  = mwld_simulator.o  random.o ../lemon-1.3/lemon/gurobi.o ../lemon-1.3/lemon/base.o ../lemon-1.3/lemon/lp_base.o ../lemon-1.3/lemon/lp_skeleton.o ../lemon-1.3/lemon/random.o ../lemon-1.3/lemon/arg_parser.o  
OBJ1  = demo_restricted.o restricted_common_nodes.o
OBJ = $(OBJ0) $(OBJ1)  
LIBS =   $(CLNFLAGS)  
CXXINCS = $(LEMONDIR) $(BOOSTDIR) 
BIN  = demo_restricted
CXXFLAGS = $(CXXINCS) $(CFLAGS_) -D HTML_LOG
RM = rm -f

.PHONY: all all-before all-after clean clean-custom

all: all-before $(BIN) all-after

clean: clean-custom
	${RM} $(OBJ) $(BIN)

demo_restricted: $(OBJ)
	$(CPP) $(OBJ0) $(OBJ1) -o "demo_restricted" $(LIBS)

random.o: random.h random.cpp
	$(CPP) -c random.cpp -o random.o $(CXXFLAGS)

../lemon-1.3/lemon/lp_base.o: ../lemon-1.3/lemon/lp_base.cc
	$(CPP) -c ../lemon-1.3/lemon/lp_base.cc -o ../lemon-1.3/lemon/lp_base.o $(CXXFLAGS)

../lemon-1.3/lemon/gurobi.o: ../lemon-1.3/lemon/gurobi.cc
	$(CPP) -c ../lemon-1.3/lemon/gurobi.cc -o ../lemon-1.3/lemon/gurobi.o $(CXXFLAGS)

../lemon-1.3/lemon/lp_skeleton.o: ../lemon-1.3/lemon/lp_skeleton.cc
	$(CPP) -c ../lemon-1.3/lemon/lp_skeleton.cc -o ../lemon-1.3/lemon/lp_skeleton.o $(CXXFLAGS)

../lemon-1.3/lemon/base.o: ../lemon-1.3/lemon/base.cc
	$(CPP) -c ../lemon-1.3/lemon/base.cc -o ../lemon-1.3/lemon/base.o $(CXXFLAGS)

../lemon-1.3/lemon/random.o: ../lemon-1.3/lemon/random.cc
	$(CPP) -c ../lemon-1.3/lemon/random.cc -o ../lemon-1.3/lemon/random.o $(CXXFLAGS)

../lemon-1.3/lemon/arg_parser.o: ../lemon-1.3/lemon/arg_parser.cc
	$(CPP) -c ../lemon-1.3/lemon/arg_parser.cc -o ../lemon-1.3/lemon/arg_parser.o $(CXXFLAGS)