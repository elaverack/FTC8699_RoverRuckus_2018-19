package org.firstinspires.ftc.teamcode.robotHandlers;

public class robotConfig {

    private robotDrive robotDrive;

    private robotServos servos = null;
    private final boolean HAS_SERVOS;

    private robotSensors sensors = null;
    private final boolean HAS_SENSORS = false;

    public robotConfig (robotDrive d) {

        robotDrive = d;
        HAS_SERVOS = false;
        //HAS_SENSORS = false;

    }
    public robotConfig (robotDrive d, robotServos s) {

        robotDrive = d;
        servos = s;
        HAS_SERVOS = true;
        //HAS_SENSORS = false;

    }
    /* public robotConfig (robotDrive d, robotServos s, robotSensors sen) {

        robotDrive = d;
        servos = s;
        HAS_SERVOS = true;
        sensors = sen;
        HAS_SENSORS = true;

    } */

    public robotDrive getRobotDrive() {return robotDrive;}
    public robotServos getServos () {return servos;}
    public boolean hasServos () {return HAS_SERVOS;}
    //public robotSensors getSensors () {return sensors;}
    public boolean hasSensors () {return HAS_SENSORS;}

}
