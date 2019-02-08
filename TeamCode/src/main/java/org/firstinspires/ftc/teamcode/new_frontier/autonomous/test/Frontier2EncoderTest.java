package org.firstinspires.ftc.teamcode.new_frontier.autonomous.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Enums;
import org.firstinspires.ftc.teamcode.frontier.control.*;
import org.firstinspires.ftc.teamcode.frontier.control.Frontier;
import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;

// Created on 1/26/2019 at 4:20 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.frontier.teleop.test

@Autonomous(name = "Frontier2EncoderTest", group = "test")
//@Disabled
public class Frontier2EncoderTest extends LinearOpMode {
    
    org.firstinspires.ftc.teamcode.frontier.control.Frontier robot;
    BNO055IMU imu;
    
    @Override
    public void runOpMode() {
        
        robot = new Frontier(this, Enums.FieldPosition.DEPOT);
    
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
    
        imu = hardwareMap.get(BNO055IMU.class, "imul");
        imu.initialize(parameters);
    
        while (!imu.isGyroCalibrated()) {
            telemetry.addData("Status", "Calibrating gyro...");
            telemetry.update();
        }
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        double start = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        
        //robot.strafeInches(24, .5, true);
        robot.turnDegrees(90, .3, true);
        
        double end = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        
        while (opModeIsActive()) {
            
            telemetry.addData("error", end - start);
            telemetry.addData("Status", "Done.");
            telemetry.update();
            
        }
    }
}