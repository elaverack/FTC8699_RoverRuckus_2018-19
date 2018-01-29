package org.firstinspires.ftc.teamcode.mecanlift.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;

// Created on 1/27/2018 at 4:42 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.autonomous

@TeleOp(name = "Alignment Lines Test", group = "Iterative Opmode")
//@Disabled
public class LinesTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VisualsHandler visuals;

    @Override
    public void init() {
        visuals = new VisualsHandler(this, false);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() { runtime.reset(); try {visuals.showAlignmentLines();} catch (InterruptedException e) { /*meh*/ } }

    @Override
    public void loop() {
        try {visuals.tryAlignmentLines(gamepad1.right_stick_x, gamepad1.right_stick_y);} catch (InterruptedException e) { /*meh*/ }
        telemetry.addData("lines", String.format("x: %1$s, y: %2$s", visuals.vertx, visuals.hory ));
        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() { visuals.close(); }

}