COMPONENT = AMWatcherAppC
BUILD_EXTRA_DEPS += AMWatcherC.class
CLEAN_EXTRA = *.class TankMsg.java DebugMsg.java
CFLAGS += -I$(TOSDIR)/lib/net \
                   -I$(TOSDIR)/lib/net/le \
                   -I$(TOSDIR)/lib/net/ctp
AMWatcherC.class: $(wildcard *.java) TankMsg.java DebugMsg.java
	javac -target 1.8 -source 1.8 *.java
TankMsg.java:
	mig java -target=null $(CFLAGS) -java-classname=TankMsg TankMessage.h TankMsg -o $(@)
DebugMsg.java:
	mig java -target=null $(CFLAGS) -java-classname=DebugMsg TankMessage.h DebugMsg -o $(@)


include $(MAKERULES)