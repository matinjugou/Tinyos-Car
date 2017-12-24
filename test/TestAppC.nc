configuration TestAppC {}
implementation {
	components MainC, LedsC, CarC;
	components TestC as App;
	App.Boot -> MainC.Boot;
	App.CarController -> CarC.CarController;
}