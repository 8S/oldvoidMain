all: imutest main motor.o communication.o hand.o leg.o AcYut.o walk.o commondefswalk.o
imutest: imutest1.cpp communication.o motor.o leg.o hand.o AcYut.o walk.o xsens/imu.o xsens/totalxsens.o commondefswalk.o
	g++ commondefswalk.o motor.o communication.o hand.o leg.o AcYut.o walk.o xsens/imu.o xsens/totalxsens.o -o imutest imutest1.cpp -lftdi -lm -lrt -lpthread
main: main.cpp communication.o motor.o leg.o hand.o AcYut.o walk.o xsens/imu.o xsens/totalxsens.o commondefswalk.o
	g++ commondefswalk.o motor.o communication.o hand.o leg.o AcYut.o walk.o xsens/imu.o xsens/totalxsens.o -o main main.cpp -lftdi -lm -lrt -lpthread
motor.o: motor.cpp motor.h commondefswalk.o
	g++ -c motor.cpp 
communication.o: communication.cpp communication.h commondefswalk.o
	g++ -c communication.cpp
leg.o: leg.cpp leg.h commondefswalk.o communication.o motor.o
	g++ -c leg.cpp
AcYut.o : motor.o communication.o leg.o AcYut.cpp AcYut.h hand.o commondefswalk.o
	g++ -c AcYut.cpp
hand.o : hand.h hand.cpp motor.o communication.o commondefswalk.o
	g++ -c hand.cpp
walk.o : walk.h walk.cpp commondefswalk.o AcYut.o xsens/imu.o xsens/totalxsens.o
	g++ -c walk.cpp
commondefswalk.o : bot1.h commondefswalk.h commondefswalk.cpp
	g++ -c commondefswalk.cpp
clean: 
	rm motor.o 
	rm communication.o
	rm leg.o
	rm AcYut.o
	rm hand.o
	rm walk.o
	rm commondefswalk.o
	rm main 

imuTest : xsens/imu.h xsens/imu.o xsens/totalxsens.o imuTest.cpp 
	g++ -o imuTest xsens/totalxsens.o xsens/imu.o imuTest.cpp -lm -lrt
cleanIMU: 
	rm imuTest
