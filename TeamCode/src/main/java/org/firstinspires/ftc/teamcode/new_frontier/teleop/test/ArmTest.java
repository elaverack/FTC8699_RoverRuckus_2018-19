package org.firstinspires.ftc.teamcode.new_frontier.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Created on 2/2/2019 at 7:16 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.new_frontier.teleop.test

@TeleOp(name = "Arm Test", group = "test")
//@Disabled
public class ArmTest extends OpMode {
    
    DcMotor shoulder, elbow;
    boolean rbd = false, lbd = false, xd = false;
    int sho_dir = 1, elb_dir = 1;
    
    @Override
    public void init() {
        
        shoulder = hardwareMap.dcMotor.get("shoulder");
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
        elbow = hardwareMap.dcMotor.get("elbow");
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        
        if (!rbd && gamepad1.right_bumper) {
            elb_dir *= -1;
            rbd = true;
        } else if (rbd && !gamepad1.right_bumper) rbd = false;
        
        if (!lbd && gamepad1.left_bumper) {
            sho_dir *= -1;
            lbd = true;
        } else if (lbd && !gamepad1.left_bumper) lbd = false;
        
        if (!xd && gamepad1.x) {
            shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            xd = false;
        } else if (xd && !gamepad1.x) xd = false;
        
        shoulder.setPower(gamepad1.left_trigger * sho_dir);
        elbow.setPower(gamepad1.right_trigger * elb_dir);
        
        telemetry.addData("shoulder", shoulder.getCurrentPosition());
        telemetry.addData("elbow", elbow.getCurrentPosition());
        
    }

    @Override
    public void stop() {
        
        shoulder.setPower(0);
        elbow.setPower(0);
        
    }

}