package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Chandler on 2/19/2017.
 */

public class robotHandler {
    //TODO: Add more as needed

    private robotConfig config;
    private LinearOpMode opMode;

    public robotHandler (LinearOpMode opMode, robotConfig config) {
        this.opMode = opMode;
        this.config = config;
    }
    public robotHandler (LinearOpMode opMode, controlConfig config, boolean slow) {
        this.opMode = opMode;
        this.config = new robotConfig(new robotDrive(opMode.hardwareMap, config, slow));
    }
    public robotHandler (LinearOpMode opMode, boolean hasEncoders, controlConfig config) {
        this.opMode = opMode;
        this.config = new robotConfig(new robotDrive(opMode.hardwareMap, hasEncoders, config));
    }
    public robotHandler (LinearOpMode opMode, boolean hasEncoders, controlConfig config, boolean slow) {
        this.opMode = opMode;
        this.config = new robotConfig(new robotDrive(opMode.hardwareMap, hasEncoders, config, slow));
    }
    public robotHandler (LinearOpMode opMode, controlConfig config, boolean slow, robotServos servos) {
        this.opMode = opMode;
        this.config = new robotConfig(new robotDrive(opMode.hardwareMap, config, slow), servos);
    }
    public robotHandler (LinearOpMode opMode, boolean hasEncoders, controlConfig config, robotServos servos) {
        this.opMode = opMode;
        this.config = new robotConfig(new robotDrive(opMode.hardwareMap, hasEncoders, config), servos);
    }
    public robotHandler (LinearOpMode opMode, boolean hasEncoders, controlConfig config, boolean slow, robotServos servos) {
        this.opMode = opMode;
        this.config = new robotConfig(new robotDrive(opMode.hardwareMap, hasEncoders, config, slow), servos);
    }

    public void drive (Gamepad gamepad) {config.getRobotDrive().drive(gamepad);}
    public void drive (Gamepad gamepad, float slow) {config.getRobotDrive().drive(gamepad, slow);}

}

