package org.firstinspires.ftc.teamcode.new_frontier.teleop.disabled;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

// Created on 1/6/2019 at 12:24 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.frontier.teleop.test

@TeleOp(name = "Roller Test", group = "test")
@Disabled
public class RollerTest extends OpMode {
    
    ModernRoboticsI2cColorSensor c;
    Servo s;
    boolean grabbed = false, doa = false;
    
    @Override
    public void init() {
        
        c = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color");
        c.enableLed(true);
        
        s = hardwareMap.servo.get("servo");
        s.setPosition(.5);
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        s.setPosition(.7);
    }

    @Override
    public void loop() {
        
        if (c.read8(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX) != 0 && !grabbed) {
            s.setPosition(.5);
            grabbed = true;
        }
        if (grabbed && gamepad1.a) {
            s.setPosition(.2);
            doa = true;
        }
        if (grabbed && !gamepad1.a && doa) {
            s.setPosition(.7);
            doa = false;
            grabbed = false;
        }
        
        telemetry.addData("color", c.read8(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX));
        if (grabbed) {
            if (c.read8(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX) > 50) telemetry.addData("type", "gold");
            else telemetry.addData("type", "silver");
        }
        telemetry.addData("grabbed", grabbed);
        telemetry.addData("doa", doa);
        
    }

    @Override
    public void stop() {
        
        c.enableLed(false);
        s.setPosition(.5);
        
    }

}