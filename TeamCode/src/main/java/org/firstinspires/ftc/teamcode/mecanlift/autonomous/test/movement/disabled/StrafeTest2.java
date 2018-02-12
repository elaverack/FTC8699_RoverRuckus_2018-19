package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test.movement.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Mecanlift;
import org.firstinspires.ftc.teamcode.visuals.Vector3;

// Created on 2/9/2018 at 3:48 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test

@Autonomous(name = "Strafe Test 2 (1ft+drift, left)", group = "test")
@Disabled
public class StrafeTest2 extends LinearOpMode {

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

        Vector3 drift = robot.strafeDistanceDrift(12,false);
        int end_theta = robot.theta();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Done. Run Time: " + runtime.toString());
            telemetry.addData("start Θ", start_theta);
            telemetry.addData("end Θ", end_theta);
            drift.teleout(this, "drift");
            telemetry.update();
        }


    }
}
