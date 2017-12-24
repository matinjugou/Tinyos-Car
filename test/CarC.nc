module CarC {
    provides {
        interface CarController;
    }
}
implementation {
    command void CarController.move() {
        return;
    }
}