package org.firstinspires.ftc.teamcode.mecanlift.autonomous.test.alignment.disabled;

import android.graphics.Bitmap;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;
import org.opencv.core.Point;

// Created on 2/23/2018 at 10:37 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift.autonomous.test.alignment

@Autonomous(name = "Cryptobox Camera", group = "test")
@Disabled
public class CryptoboxPics extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private VisualsHandler visuals;
    private int cd = -1, picID = 0;

    @Override
    public void init() {
        visuals = new VisualsHandler(this, false);
        VisualsHandler.phoneLightOn();

        while (visuals.getPreview() == null) {
            telemetry.addData("Status", "Waiting");
            telemetry.update();
        }

        visuals.getPreview().setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) { visuals.takeAndSavePic("cryptobox_test.png"); picID++; }
        });

        telemetry.addData("Status", "Initialized");
        runtime.reset();
    }

    @Override
    public void init_loop() {
        //if (((int)(runtime.seconds()%2) == 1)) visuals.showAlignmentCircles();
    }

    @Override
    public void start() {
        runtime.reset(); //cd = visuals.vuforia.switchToFrontCamera();
    }

    @Override
    public void loop() {
        //if (((int)(runtime.seconds()%3) == 1)) visuals.vuforia.takeFrontPic();
//        Bitmap bitmap = ((FtcRobotControllerActivity)hardwareMap.appContext).frontFrame;
//        if (bitmap != null) visuals.setPreview(bitmap);
        //if (visuals.vuforia.frontFrame != null) visuals.setPreview(visuals.vuforia.frontFrame);
        if (((int)(runtime.milliseconds()%500) == 0)) visuals.previewVuforia();

        telemetry.addData("Status", "Saved " + picID);
//        telemetry.addData("Status", "Running: " + runtime.toString());
//        telemetry.addData("meh", cd);
    }

    @Override
    public void stop() { visuals.close(); }

}