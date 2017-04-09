package org.firstinspires.ftc.teamcode.robotHandlers;

public class RobotConfig {

    @Deprecated private StandardRobotDrive robotDrive;
    @Deprecated private RobotServos servos = null;
    @Deprecated private RobotSensors sensors = null;

    @Deprecated public RobotConfig(StandardRobotDrive d) {

        robotDrive = d;
        HAS_SERVOS = false;
        HAS_SENSORS = false;

        NO_WHEELS = 0; NO_SERVOS = 0; NO_SENSORS = 0; DRIVE_TYPE = driveType.TANK;

    }
    @Deprecated public RobotConfig(StandardRobotDrive d, RobotServos s) {

        robotDrive = d;
        servos = s;
        HAS_SERVOS = true;
        HAS_SENSORS = false;

        NO_WHEELS = 0; NO_SERVOS = 0; NO_SENSORS = 0; DRIVE_TYPE = driveType.TANK;

    }
    @Deprecated public RobotConfig (StandardRobotDrive d, RobotServos s, RobotSensors sen) {

        robotDrive = d;
        servos = s;
        HAS_SERVOS = true;
        sensors = sen;
        HAS_SENSORS = true;

        NO_WHEELS = 0; NO_SERVOS = 0; NO_SENSORS = 0; DRIVE_TYPE = driveType.TANK;

    }

    @Deprecated public StandardRobotDrive getRobotDrive() {return robotDrive;}
    @Deprecated public RobotServos getServos () {return servos;}
    @Deprecated public RobotSensors getSensors () {return sensors;}


    final boolean
            HAS_SERVOS,
            HAS_SENSORS;

    final int
            NO_WHEELS,
            NO_SERVOS,
            NO_SENSORS;

    final driveType DRIVE_TYPE;

    public enum driveType {TANK, MECANUM, HYBRID, OTHER}

    // Default constructor for default robot
    public RobotConfig() {
        HAS_SERVOS = false; HAS_SENSORS = false;
        NO_WHEELS = 0; NO_SERVOS = 0; NO_SENSORS = 0;

        DRIVE_TYPE = driveType.TANK;
    }
    public RobotConfig(int noWheels, driveType drType) {
        HAS_SERVOS = false; HAS_SENSORS = false;
        NO_WHEELS = noWheels; NO_SERVOS = 0; NO_SENSORS = 0;

        DRIVE_TYPE = drType;
    }
    public RobotConfig(int noWheels, driveType drType, boolean numIsServos, int number) {
        HAS_SERVOS = numIsServos; HAS_SENSORS = !numIsServos;
        if (numIsServos) {NO_SERVOS = number; NO_SENSORS = 0;}
            else {NO_SERVOS = 0; NO_SENSORS = number;}

        NO_WHEELS = noWheels;
        DRIVE_TYPE = drType;
    }
    public RobotConfig(int noWheels, driveType drType, int noServos, int noSensors) {
        HAS_SERVOS = true; HAS_SENSORS = true;
        NO_WHEELS = noWheels; NO_SERVOS = noServos; NO_SENSORS = noSensors;

        DRIVE_TYPE = drType;
    }

}
