package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Mecanlift;
import org.firstinspires.ftc.teamcode.visuals.Vector3;
import org.firstinspires.ftc.teamcode.visuals.VuforiaHandler;

// Created on 2/10/2018 at 2:27 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test

@Autonomous(name = "RC_TEST_AUTO", group = "test")
//@Disabled
public class RC_TEST_AUTO extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Mecanlift robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Mecanlift(this, Mecanlift.Color.RED, Mecanlift.Position.CORNER);
        robot.calibrateGyro();
        robot.activateVuforia();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        while (!isStarted()) {
            telemetry.addData("Seeing?", robot.checkColumn());
            telemetry.update();
        }
        runtime.reset();

        robot.doJewels();

        VuforiaHandler.PosRot pr = robot.driveOffBalance(this);

        robot.driveToCryptobox(this);

        robot.placeGlyph(this);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            robot.tellDriverRelPos();
            telemetry.addData("Column", robot.keyColumn());
            telemetry.addData("Theta", robot.theta());
            telemetry.addData("Y", pr.rotation.ry());
            pr.position.teleout(this, "pos");
            telemetry.update();
        }

        robot.closeVisuals();

    }
}
