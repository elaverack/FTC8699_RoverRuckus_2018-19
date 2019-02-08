package org.firstinspires.ftc.teamcode.new_frontier.teleop.test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

// Created on 2/6/2019 at 10:11 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.new_frontier.teleop.test

@TeleOp(name = "Limit Switch Test", group = "test")
//@Disabled
public class LimitTest extends OpMode {
    
    ModernRoboticsTouchSensor t_shoulder, t_elbow;
    
    @Override
    public void init() {
        
        t_shoulder = (ModernRoboticsTouchSensor)hardwareMap.touchSensor.get("slim");
        
        t_elbow = (ModernRoboticsTouchSensor)hardwareMap.touchSensor.get("elim");
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        
        if (t_shoulder.isPressed()) telemetry.addData("shoulder", "pressed");
        else telemetry.addData("shoulder", "not pressed");

        if (t_elbow.isPressed()) telemetry.addData("elbow", "pressed");
        else telemetry.addData("elbow", "not pressed");
        
//        telemetry.addData("shoulder", t_shoulder.getState());
//        telemetry.addData("elbow", t_elbow.getState());
        
    }

    @Override
    public void stop() {
        
        
        
    }

}