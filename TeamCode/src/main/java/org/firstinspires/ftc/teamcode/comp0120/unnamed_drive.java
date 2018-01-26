package org.firstinspires.ftc.teamcode.comp0120;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 1/14/2018 at 3:46 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "unnamed_drive", group = "Iterative Opmode")
@Disabled
public class unnamed_drive extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private unnamed robot;

    @Override
    public void init() {
        robot = new unnamed(this);

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

        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() {
        robot.stop();
    }

}
