all:ser cli
cli:p2pclient.cpp
	 g++ -std=c++0x $^ -o $@ -lpthread -lboost_filesystem -lboost_system -lboost_thread
ser:p2pserver.cpp
	 g++ -std=c++0x $^ -o $@ -lpthread -lboost_filesystem -lboost_system
