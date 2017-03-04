package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Chandler on 2/19/2017.
 */

public class RobotHandler {
    //TODO: Add more as needed

    private RobotConfig config;
    private LinearOpMode opMode;

    public RobotHandler(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;
    }
    public RobotHandler(LinearOpMode opMode, ControlConfig config, boolean slow) {
        this.opMode = opMode;
        this.config = new RobotConfig(new RobotDrive(opMode.hardwareMap, config, slow));
    }
    public RobotHandler(LinearOpMode opMode, boolean hasEncoders, ControlConfig config) {
        this.opMode = opMode;
        this.config = new RobotConfig(new RobotDrive(opMode.hardwareMap, hasEncoders, config));
    }
    public RobotHandler(LinearOpMode opMode, boolean hasEncoders, ControlConfig config, boolean slow) {
        this.opMode = opMode;
        this.config = new RobotConfig(new RobotDrive(opMode.hardwareMap, hasEncoders, config, slow));
    }
    public RobotHandler(LinearOpMode opMode, ControlConfig config, boolean slow, RobotServos servos) {
        this.opMode = opMode;
        this.config = new RobotConfig(new RobotDrive(opMode.hardwareMap, config, slow), servos);
    }
    public RobotHandler(LinearOpMode opMode, boolean hasEncoders, ControlConfig config, RobotServos servos) {
        this.opMode = opMode;
        this.config = new RobotConfig(new RobotDrive(opMode.hardwareMap, hasEncoders, config), servos);
    }
    public RobotHandler(LinearOpMode opMode, boolean hasEncoders, ControlConfig config, boolean slow, RobotServos servos) {
        this.opMode = opMode;
        this.config = new RobotConfig(new RobotDrive(opMode.hardwareMap, hasEncoders, config, slow), servos);
    }

    public void drive (Gamepad gamepad) {config.getRobotDrive().drive(gamepad);}
    public void drive (Gamepad gamepad, float slow) {config.getRobotDrive().drive(gamepad, slow);}

}

