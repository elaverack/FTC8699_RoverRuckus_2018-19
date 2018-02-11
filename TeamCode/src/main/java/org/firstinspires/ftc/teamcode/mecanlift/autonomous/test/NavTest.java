package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.VuforiaHandler;

// Created on 1/27/2018 at 3:22 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.autonomous

@Autonomous(name = "Navigation Test", group = "test")
//@Disabled
public class NavTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaHandler vuforia;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        vuforia = new VuforiaHandler(this);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() { runtime.reset(); vuforia.start(); }

    @Override
    public void loop() {
        vuforia.tellDriverRelPos();
        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() {  }

}