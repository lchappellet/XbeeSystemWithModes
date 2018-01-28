#Makefile to run SignalLocator files.
#still not done. IN PROGRESS.......


objects = main.o MotorControl.o EncoderXBeesystem.o arduino_hal.o
INC =-I/home/londoncv/arduino-1.8.5/hardware/arduino/avr/cores/arduino/

mainSignal : $(objects)
	cc -o mainSignal $(objects)
main.o : main.cpp MotorControl.h
	cc $(INC) -c main.cpp
MotorControl.o : MotorControl.cpp MotorControl.h
	cc $(INC) -c MotorControl.cpp
arduino_hal.o : arduino_hal.cpp arduino_hal.h hal.h
	cc -c arduino_hal.cpp
EncoderXBeesystem.o : EncoderXBeesystem..cpp Encoder.h EncoderXBeesystem.h
	cc -c EncoderXBeesystem.cpp

.PHONY : clean
clean :
	-rm mainSignal $(objects)
