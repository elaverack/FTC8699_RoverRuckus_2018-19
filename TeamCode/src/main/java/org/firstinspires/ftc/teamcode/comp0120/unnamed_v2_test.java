package org.firstinspires.ftc.teamcode.comp0120;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 1/20/2018 at 1:31 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "qual2_ATEST", group = "Iterative Opmode")
@Disabled
public class unnamed_v2_test extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private unnamed_v2 robot;

    @Override
    public void init() {
        robot = new unnamed_v2(this);
        robot.drive_init();
        robot.auto_init();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();

        robot.start();
    }

    @Override
    public void loop() {
        robot.drive();
        robot.drive_jewel();
        robot.sensorOut();

        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() {
        robot.stop();
    }

}
