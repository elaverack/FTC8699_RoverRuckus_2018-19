package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test.jewels;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;

// Created on 2/25/2018 at 2:53 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test.jewels

@Autonomous(name = "Preview Jewels", group = "test")
//@Disabled
public class PreviewJewels extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VisualsHandler visuals;

    @Override
    public void init() {
        visuals = new VisualsHandler(this, false);
        VisualsHandler.phoneLightOn();

        telemetry.addData("Status", "Initialized");
        runtime.reset();
    }

    @Override
    public void init_loop() {
        if (((int)(runtime.seconds()%2) == 1)) visuals.showAlignmentCircles();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (((int)(runtime.seconds()%3) == 0)) visuals.previewJewels();

        telemetry.addData("Status", "Running: " + runtime.toString());
    }

    @Override
    public void stop() { visuals.close(); }

}