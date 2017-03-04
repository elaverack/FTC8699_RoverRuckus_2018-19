package org.firstinspires.ftc.teamcode.robotHandlers;

public class RobotConfig {

    private RobotDrive robotDrive;

    private RobotServos servos = null;
    private final boolean HAS_SERVOS;

    private RobotSensors sensors = null;
    private final boolean HAS_SENSORS = false;

    public RobotConfig(RobotDrive d) {

        robotDrive = d;
        HAS_SERVOS = false;
        //HAS_SENSORS = false;

    }
    public RobotConfig(RobotDrive d, RobotServos s) {

        robotDrive = d;
        servos = s;
        HAS_SERVOS = true;
        //HAS_SENSORS = false;

    }
    /* public RobotConfig (RobotDrive d, RobotServos s, RobotSensors sen) {

        RobotDrive = d;
        servos = s;
        HAS_SERVOS = true;
        sensors = sen;
        HAS_SENSORS = true;

    } */

    public RobotDrive getRobotDrive() {return robotDrive;}
    public RobotServos getServos () {return servos;}
    public boolean hasServos () {return HAS_SERVOS;}
    //public RobotSensors getSensors () {return sensors;}
    public boolean hasSensors () {return HAS_SENSORS;}

}
