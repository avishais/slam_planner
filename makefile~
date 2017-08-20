#compiler
OMPL_DIR = /usr/local

INC_PLANNERS = /home/avishai/Downloads/omplapp/ompl/Workspace/slam/planners/
INC_VALIDITY = /home/avishai/Downloads/omplapp/ompl/Workspace/slam/validity_checkers/
INC_RUN = /home/avishai/Downloads/omplapp/ompl/Workspace/slam/run/

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu
LDFLAGS=  -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app_base -lompl_app -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

CPPS = ${INC_RUN}plan.cpp ${INC_VALIDITY}collisions.cpp ${INC_VALIDITY}StateValidityChecker.cpp ${INC_PLANNERS}myRRT.cpp 

all:
	$(CXX) $(CPPS) -o psl $(CXXFLAGS) $(LDFLAGS) -std=c++11 #-Wall
	#$(CXX) ${INC_VALIDITY}collisions.cpp -o co -std=c++11 #-Wall



