package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Mecanlift;
import org.firstinspires.ftc.teamcode.visuals.Vector3;

// Created on 2/8/2018 at 9:11 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test

@Autonomous(name = "Drift Test (1ft)", group = "test")
//@Disabled
public class DriftTest extends LinearOpMode {

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

        Vector3 drift = robot.driveDistanceDrift(12.0);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drift", String.format("x: %1$s, y: %2$s, z: %3$s",
                    drift.rx(),
                    drift.ry(),
                    drift.rz()
            ));
            telemetry.update();
        }


    }
}
