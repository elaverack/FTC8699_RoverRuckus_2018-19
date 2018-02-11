package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Mecanlift;

// Created on 2/9/2018 at 3:48 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test

@Autonomous(name = "Strafe Test Long (11200, left)", group = "test")
//@Disabled
public class StrafeTestL extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Mecanlift robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Mecanlift(this);
        robot.calibrateGyro();
        int start_theta = robot.theta();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        robot.strafeDistance(11200,false);
        int end_theta = robot.theta();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Done. Run Time: " + runtime.toString());
            telemetry.addData("start Θ", start_theta);
            telemetry.addData("end Θ", end_theta);
            telemetry.update();
        }


    }
}
