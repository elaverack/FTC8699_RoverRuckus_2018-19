package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Chandler on 2/19/2017.
 */

public abstract class RobotHandler {
    //TODO: Add more as needed

    @Deprecated public StandardRobotDrive getRobotDrive() {return config.getRobotDrive();}
    @Deprecated public RobotServos getServos () {return config.getServos();}
    @Deprecated public RobotSensors getSensors () {return config.getSensors();}

    protected RobotConfig config;
    public StandardRobotDrive drive;
    public RobotServos servos = null;
    public RobotSensors sensors = null;

    protected RobotHandler(){}
    public RobotHandler(RobotConfig config) {this.config = config;}

    public abstract void drive();

    public void stop() {drive.stopAll();}

    public int numberOfWheels () {return config.NO_WHEELS;}
    public RobotConfig.driveType driveType () {return config.DRIVE_TYPE;}

    public boolean hasServos () {return config.HAS_SERVOS;}
    public int numberOfServos () {return config.NO_SERVOS;}

    public boolean hasSensors () {return config.HAS_SENSORS;}
    public int numberOfSensors () {return config.NO_SENSORS;}

}

