package org.firstinspires.ftc.teamcode.new_frontier.teleop.test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.new_frontier.control.Arm;

// Created on 2/7/2019 at 7:44 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.new_frontier.teleop.test

@TeleOp(name = "Arm Homing Test", group = "test")
//@Disabled
public class HomingTest extends LinearOpMode {
    
    Arm a;
    
    @Override
    public void runOpMode() {
        
        a = new Arm(
                hardwareMap.dcMotor.get("shoulder"),
                hardwareMap.dcMotor.get("elbow"),
                (ModernRoboticsTouchSensor) hardwareMap.touchSensor.get("slim"),
                (ModernRoboticsTouchSensor) hardwareMap.touchSensor.get("elim"),
                this);
        
        telemetry.addData("Status", "Homing arm...");
        telemetry.update();
        
        a.init();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        while (opModeIsActive()) {
    
            a.drive(gamepad1.a, gamepad1.dpad_left, gamepad1.b, gamepad1.y, gamepad1.x,
                    gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.right_stick_button,
                    gamepad1.left_trigger, gamepad1.right_trigger);
            
            a.telemetry();
            
            telemetry.update();
            
        }
    }
}