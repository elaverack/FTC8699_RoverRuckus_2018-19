package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Mecanlift;

// Created on 2/10/2018 at 11:52 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test

@Autonomous(name = "Rotate Test (90)", group = "test")
//@Disabled
public class RotateTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Mecanlift robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Mecanlift(this);
        robot.calibrateGyro();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        robot.turnToAngle(90,true, .5f);
        runtime.reset();
        while (runtime.seconds() < 3);
        int theta = robot.theta();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Done. Run Time: " + runtime.toString());
            telemetry.addData("theta", theta);
            telemetry.update();
        }


    }
}
