package org.firstinspires.ftc.teamcode.frontier.autonomous.test.disabled;

import android.graphics.PointF;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.Graph;
import org.opencv.core.Scalar;

import java.util.ArrayList;

// Created on 9/12/2018 at 10:12 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@Autonomous(name = "DriveErrorGraph", group = "test")
@Disabled
public class GraphTest3 extends LinearOpMode {
    
    Graph g, g2;
    DcMotor lf, lb, rf, rb;
    ModernRoboticsI2cGyro gyro;
    
    @Override
    public void runOpMode() {
    
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
    
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
    
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        
        g = new Graph(
                "Motor Proportional Curve, Stock PID, 100%, 1 rot., 40:1",
                "Time (seconds)",
                "Min. Encoder over Encoder Count",
                1200,
                720,
                0,
                2.5,
                .5,
                0,
                1.1f,
                .1f);
    
        g2 = new Graph(
                "Gyro angle, Stock PID, 100%, 1 rot., 40:1",
                "Time (seconds)",
                "Gyro readout (degrees)",
                1200,
                720,
                0,
                2.5,
                .5,
                -10,
                10,
                1);
    
        ArrayList<PointF>
                rfp = new ArrayList<>(),
                rbp = new ArrayList<>(),
                lfp = new ArrayList<>(),
                lbp = new ArrayList<>(),
                gp = new ArrayList<>();
    
        ElapsedTime t = new ElapsedTime();
    
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            telemetry.addData("Status", "Calibrating gyro...");
            telemetry.update();
        }
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        rf.setTargetPosition(1120);
        rb.setTargetPosition(1120);
        lf.setTargetPosition(1120);
        lb.setTargetPosition(1120);
        rf.setPower(1);
        rb.setPower(1);
        lf.setPower(1);
        lb.setPower(1);
        
        t.reset();
        
        while (t.seconds() <= 2.5) {
            int
                    erf = 1120 - rf.getCurrentPosition(),
                    erb = 1120 - rb.getCurrentPosition(),
                    elf = 1120 - lf.getCurrentPosition(),
                    elb = 1120 - lb.getCurrentPosition(),
                    slow = Math.max(Math.max(erf, erb), Math.max(elf, elb));
            if (slow != 0 && Math.min(Math.min(erf, erb), Math.min(elf, elb)) >= 0) {
                rfp.add(new PointF((float) t.seconds(), Math.abs((float) erf / slow)));
                rbp.add(new PointF((float) t.seconds(), Math.abs((float) erb / slow)));
                lfp.add(new PointF((float) t.seconds(), Math.abs((float) elf / slow)));
                lbp.add(new PointF((float) t.seconds(), Math.abs((float) elb / slow)));
            }
            gp.add(new PointF((float)t.seconds(), theta()));
        }
    
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        
        g.plot(rfp, 2, new Scalar(255,0,0));
        g.plot(rbp, 2, new Scalar(0,255,0));
        g.plot(lfp, 2, new Scalar(0,0,255));
        g.plot(lbp, 2, new Scalar(255,145,0));
        g.referenceLine(new PointF(0,0), new PointF(2.5f,0), 1, new Scalar(128,128,128));
        
        g2.plot(gp, 2, new Scalar(255,0,0));
        g2.referenceLine(new PointF(0,0), new PointF(2.5f, 0), 1, new Scalar(128,128,128));
        
        g.saveGraph("stock pid error at 100 for 1 rot.png");
        g2.saveGraph("stock pid error at 100 for 1 rot gyro.png");
        
        while (opModeIsActive()) {
            
            telemetry.addData("Status", "Done.");
            telemetry.update();
            
        }
    }
    
    private int theta() {
        int theta = gyro.getHeading();
        if (theta > 180) theta -= 360;
        return theta;
    }
}