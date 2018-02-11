package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Mecanlift;

// Created on 2/3/2018 at 12:04 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous

@Autonomous(name = "Lines Test 2 (aligning test)", group = "test")
//@Disabled
public class LinesTest2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Mecanlift robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Mecanlift(this, Mecanlift.Color.BLUE, Mecanlift.Position.JEWEL_ONLY);

        telemetry.addData("Status", "Align vert. line with edge of jewel and hor. line with bottom of vumark.");
        telemetry.update();
        while (!isStarted()) {
            robot.showAligning();
        }
        runtime.reset();

        robot.doParkAutonomous(this);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }


    }
}
