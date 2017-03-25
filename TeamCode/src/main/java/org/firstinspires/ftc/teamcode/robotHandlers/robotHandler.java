package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Chandler on 2/19/2017.
 */

public abstract class RobotHandler {
    //TODO: Add more as needed

    private RobotConfig config;

    RobotHandler(){}
    public RobotHandler(RobotConfig config) {this.config = config;}

    public abstract void drive();

    public StandardRobotDrive getRobotDrive() {return config.getRobotDrive();}
    public RobotServos getServos () {return config.getServos();}
    public boolean hasServos () {return config.hasServos();}
    public RobotSensors getSensors () {return config.getSensors();}
    public boolean hasSensors () {return config.hasSensors();}
    public void stop() {config.getRobotDrive().stopAll();}

}

