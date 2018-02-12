package org.firstinspires.ftc.teamcode.mecanlift.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Mecanlift;

// Created on 2/10/2018 at 2:27 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test

@Autonomous(name = "QUAL_BC_AUTO", group = "test")
//@Disabled
public class BC_AUTO extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Mecanlift robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Mecanlift(this, Mecanlift.Color.BLUE, Mecanlift.Position.CORNER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted()) { robot.doFullInitLoop(); }

        runtime.reset();

        robot.doFullAutonomous(this);

    }
}
