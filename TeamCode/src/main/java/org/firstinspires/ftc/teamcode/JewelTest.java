package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;
import org.firstinspires.ftc.teamcode.visuals.VuforiaHandler;

// Created on 11/4/2017 at 9:56 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "JewelTest", group = "Linear Opmode")
//@Disabled
public class JewelTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VisualsHandler vh;

    @Override
    public void runOpMode() throws InterruptedException {

        vh = new VisualsHandler(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        vh.OpenCVTest();

        while (opModeIsActive()) {


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
