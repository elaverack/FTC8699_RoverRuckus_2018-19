package org.firstinspires.ftc.teamcode.frontier.autonomous.test;

import android.graphics.Bitmap;
import android.view.View;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

// Created on 10/14/2018 at 12:52 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "Sample Test", group = "test")
//@Disabled
public class SampleTest extends OpMode {
    
    @Override
    public void init() {
    
        ImageView iv = VisualsHandler.tempSetup(this);
        
        VisualsHandler.phoneLightOn();
        
        VisualsHandler.takeAndPreviewPicture();
        
        iv.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                telemetry.addData("Arr", VisualsHandler.takePreviewAndSample());
            }
        });
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        
        //telemetry.addData("Status", "Running");
        
    }

    @Override
    public void stop() {
        
        VisualsHandler.tempSetupStop();
        
    }

}