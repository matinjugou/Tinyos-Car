COMPONENT = TankBrainAppC
BUILD_EXTRA_DEPS += TankBrainC.class
CLEAN_EXTRA = *.class TankMsg.java
CFLAGS += -I$(TOSDIR)/lib/net \
                   -I$(TOSDIR)/lib/net/le \
                   -I$(TOSDIR)/lib/net/ctp
TankBrainC.class: $(wildcard *.java) TankMsg.java
	javac -target 1.8 -source 1.8 *.java
TankMsg.java:
	mig java -target=null $(CFLAGS) -java-classname=TankMsg TankMessage.h TankMsg -o $(@)
include $(MAKERULES)