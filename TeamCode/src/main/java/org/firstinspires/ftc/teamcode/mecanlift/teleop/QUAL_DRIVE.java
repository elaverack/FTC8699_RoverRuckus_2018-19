package org.firstinspires.ftc.teamcode.mecanlift.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Mecanlift;

// Created on 1/29/2018 at 3:41 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.teleop

@TeleOp(name = "QUAL_DRIVE", group = "Iterative Opmode")
//@Disabled
public class QUAL_DRIVE extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Mecanlift robot;

    @Override public void init() { robot = new Mecanlift(this); telemetry.addData("Status", "Initialized"); }

    @Override public void init_loop() {  }

    @Override public void start() { robot.start(); runtime.reset(); }

    @Override public void loop() { robot.drive(true); telemetry.addData("Status", "Running: " + runtime.toString()); }

    @Override public void stop() { robot.stop(); }

}