package org.firstinspires.ftc.teamcode.mecanlift.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Mecanlift;

// Created on 2/15/2018 at 9:53 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.teleop.test

@TeleOp(name = "Mecanlift Auto Drive", group = "old")
//@Disabled
public class QUAL_AUTOGRAB_DRIVE extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Mecanlift robot;
    private Thread drive;

    @Override public void init() { robot = new Mecanlift(this); drive = new Thread(new runDrive(), "drive"); telemetry.addData("Status", "Initialized"); }

    @Override public void init_loop() {  }

    @Override public void start() { robot.start(); drive.start(); runtime.reset(); }

    @Override public void loop() {
        robot.runDrive();
        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override public void stop() { drive.interrupt(); robot.stop(); }

    class runDrive implements Runnable {

        public void run() {
            while (!Thread.interrupted()) { robot.runAutoMechanisms(); }
            onStop();
        }

        private void onStop () { }

    }

}