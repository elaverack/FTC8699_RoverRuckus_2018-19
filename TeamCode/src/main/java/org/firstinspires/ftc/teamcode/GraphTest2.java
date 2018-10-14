package org.firstinspires.ftc.teamcode;

import android.graphics.PointF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.Graph;
import org.opencv.core.Scalar;

import java.util.ArrayList;

// Created on 9/14/2018 at 4:57 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "SingleMotorGraph", group = "test")
@Disabled
public class GraphTest2 extends LinearOpMode {
    
    DcMotor m;
    Graph g;
    
    @Override
    public void runOpMode() {
        
        m = hardwareMap.dcMotor.get("m");
        m.setDirection(DcMotor.Direction.FORWARD);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
        g = new Graph(
                "Motor Position Curve, Stock PID on Unspecified, 100%, 1 rot., 40:1",
                "Time (seconds)",
                "Encoder Count",
                1200,
                720,
                0,
                2.5,
                .5,
                -100,
                1200,
                100);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
    
        ArrayList<PointF> mp = new ArrayList<>();
    
        ElapsedTime t = new ElapsedTime();
    
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
    
        m.setTargetPosition(1120);
        m.setPower(1);
    
        t.reset();
    
        while (t.seconds() <= 2.5) mp.add(new PointF((float)t.seconds(), m.getCurrentPosition()));
    
        m.setPower(0);
    
        g.plot(mp, 2, new Scalar(255, 0, 0));
        g.referenceLine(new PointF(0,1120), new PointF(2.5f,1120), 1, new Scalar(128,128,128));
    
        g.saveGraph("stock pid single motor unspecified at 100 for 1 rot.png");
    
        while (opModeIsActive()) {
        
            telemetry.addData("Status", "Done.");
            telemetry.update();
        
        }
    }
}