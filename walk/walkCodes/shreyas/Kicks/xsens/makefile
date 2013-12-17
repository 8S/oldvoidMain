all: imu.o test.cpp
	g++ test.cpp totalxsens.o imu.o -o test -lrt
imu.o: imu.cpp imu.h totalxsens.o
	g++ -c imu.cpp