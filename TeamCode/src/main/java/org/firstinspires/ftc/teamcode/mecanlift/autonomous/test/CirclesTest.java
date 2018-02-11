package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;
import org.firstinspires.ftc.teamcode.visuals.VuforiaHandler;

// Created on 2/10/2018 at 1:24 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test

@Autonomous(name = "Circles Test (gamepad)", group = "test")
//@Disabled
public class CirclesTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    VisualsHandler visuals;

    @Override
    public void init() {
        visuals = new VisualsHandler(this, false);
        VisualsHandler.phoneLightOn();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset(); visuals.showAlignmentCircles();
    }

    @Override
    public void loop() {
        int redr = 0, bluer = 0;
        if (gamepad1.b) redr = 1;
        if (gamepad1.a) redr = -1;
        if (gamepad1.y) bluer = 1;
        if (gamepad1.x) bluer = -1;
        visuals.tryAlignmentCircles(
                gamepad1.right_stick_x, gamepad1.right_stick_y, redr, gamepad1.left_stick_x, gamepad1.left_stick_y, bluer
        );

        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("red", String.format("x: %1$s, y: %2$s, r: %3$s",
                visuals.red.center.x,
                visuals.red.center.y,
                visuals.red.radius
        ));
        telemetry.addData("blue", String.format("x: %1$s, y: %2$s, r: %3$s",
                visuals.blue.center.x,
                visuals.blue.center.y,
                visuals.blue.radius
        ));
    }

    @Override
    public void stop() {
        visuals.close();
    }

}