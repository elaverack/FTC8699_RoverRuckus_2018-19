package org.firstinspires.ftc.teamcode.frontier.autonomous.test;

import android.graphics.PointF;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.visuals.Graph;
import org.opencv.core.Scalar;

import java.util.HashMap;
import java.util.Map;

// Created on 12/10/2018 at 9:03 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.frontier.autonomous.test

@Autonomous(name = "Encoder Test Custom", group = "test")
//@Disabled
public class EncoderTestCustom extends LinearOpMode {
    
    private static final int countPerRev = 1440;
    private static final double inchesDis = 36.0;
    private static final String TAG = "PID_TEST";
    
    private double
            dis_kp  = .021,
            dis_ki,
            dis_kd,
            sp_kp   = .05,
            sp_ki   = 30,
            sp_kd   = 0;
    
    private static final double
            speedCutOff = 40.0,
            powerCutOff = .8;
    
    private double speedGoal = 30.0;
    
    private DcMotor rf, rb, lf, lb;
    
    @Override
    public void runOpMode() {
    
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
    
        Graph g = new Graph("Holding constant 30 rpm kp: " + sp_kp + " ki: " + sp_ki + " kd: " + sp_kd, "Time", "Speed (rpm)", 1200, 720, 0, 6, 1, 0, 100, 10);
        Graph g2 = new Graph("Power for holding constant 30 rpm kp: " + sp_kp + " ki: " + sp_ki + " kd: " + sp_kd, "Time", "Power (%)", 1200, 720, 0, 6, 1, 0, 100, 10);
        
        HashMap<Double, double[]>
                setPowers = new HashMap<>(),
                holdSpeeds = new HashMap<>();
        
        ElapsedTime t = new ElapsedTime();
    
        int rfLastPos = 0, rbLastPos = 0, lfLastPos = 0, lbLastPos = 0;
        double lastTime = 0;
        double[]
                lasterrors = {speedGoal, speedGoal, speedGoal, speedGoal},
                errorsums = {0, 0, 0, 0};
        
        t.reset();
        while (t.seconds() <= 6) {
            double dt = t.seconds() - lastTime;
            lastTime += dt;
            
            double[] speeds = {
                    ((rf.getCurrentPosition() - rfLastPos) / dt) / 24,
                    ((rb.getCurrentPosition() - rbLastPos) / dt) / 24,
                    ((lf.getCurrentPosition() - lfLastPos) / dt) / 24,
                    ((lb.getCurrentPosition() - lbLastPos) / dt) / 24
            }, errors = {
                    speedGoal - speeds[0],
                    speedGoal - speeds[1],
                    speedGoal - speeds[2],
                    speedGoal - speeds[3]
            }, pow_pds = {
                    sp_kp*errors[0] + sp_kd * ((errors[0] - lasterrors[0]) / dt),
                    sp_kp*errors[1] + sp_kd * ((errors[1] - lasterrors[1]) / dt),
                    sp_kp*errors[2] + sp_kd * ((errors[2] - lasterrors[2]) / dt),
                    sp_kp*errors[3] + sp_kd * ((errors[3] - lasterrors[3]) / dt)
            }, powers = new double[4];
            
            for (int i = 0; i < pow_pds.length; i++) {
                double
                        toAdd = errors[i] * dt,
                        power = pow_pds[i] + sp_ki * (errorsums[i] + toAdd);
                if (power > powerCutOff) {
                    powers[i] = powerCutOff;
                    if (errors[i] * power >= 0.0) continue;
                } else powers[i] = power;
                errorsums[i] += toAdd;
            }
            
            lasterrors = errors;
            
            rf.setPower(powers[0]);
            rb.setPower(powers[1]);
            lf.setPower(powers[2]);
            lb.setPower(powers[3]);
            
            setPowers.put(lastTime, powers);
            holdSpeeds.put(lastTime, speeds);
            
            rfLastPos = rf.getCurrentPosition();
            rbLastPos = rb.getCurrentPosition();
            lfLastPos = lf.getCurrentPosition();
            lbLastPos = lb.getCurrentPosition();
    
            Log.d(TAG, "dt: " + dt);
        }
        
        setPower(0);
        
        for (Map.Entry<Double, double[]> e : setPowers.entrySet()) {
            g2.plot(new PointF(e.getKey().floatValue(), (float)e.getValue()[0]), 2, new Scalar(255,0,0));
            g2.plot(new PointF(e.getKey().floatValue(), (float)e.getValue()[1]), 2, new Scalar(0,255,0));
            g2.plot(new PointF(e.getKey().floatValue(), (float)e.getValue()[2]), 2, new Scalar(0,0,255));
            g2.plot(new PointF(e.getKey().floatValue(), (float)e.getValue()[3]), 2, new Scalar(255,145,0));
        }
        for (Map.Entry<Double, double[]> e : holdSpeeds.entrySet()) {
            g.plot(new PointF(e.getKey().floatValue(), (float)e.getValue()[0]), 2, new Scalar(255,0,0));
            g.plot(new PointF(e.getKey().floatValue(), (float)e.getValue()[1]), 2, new Scalar(0,255,0));
            g.plot(new PointF(e.getKey().floatValue(), (float)e.getValue()[2]), 2, new Scalar(0,0,255));
            g.plot(new PointF(e.getKey().floatValue(), (float)e.getValue()[3]), 2, new Scalar(255,145,0));
        }
    
        g.saveGraph("custom pid speed hold 30rpm speed.png");
        g2.saveGraph("custom pid speed hold 30rpm power.png");
        
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
        rfLastPos = rf.getCurrentPosition();
        rbLastPos = rb.getCurrentPosition();
        lfLastPos = lf.getCurrentPosition();
        lbLastPos = lb.getCurrentPosition();
        
        while (opModeIsActive()) {
    
            double dt = t.seconds() - lastTime;
            lastTime = t.seconds();
    
            float powerRF, powerRB, powerLF, powerLB;
            float
                    straight    = -gamepad1.left_stick_y,
                    strafe      = gamepad1.left_stick_x,
                    rotate      = gamepad1.right_stick_x;
            powerRF = straight;
            powerRB = straight;
            powerLF = straight;
            powerLB = straight;
            powerRF -= strafe;
            powerRB += strafe;
            powerLF += strafe;
            powerLB -= strafe;
            powerRF -= rotate;
            powerRB -= rotate;
            powerLF += rotate;
            powerLB += rotate;
            if (powerRF > 1) {powerRF = 1f;} else if (powerRF < -1) {powerRF = -1f;}
            if (powerRB > 1) {powerRB = 1f;} else if (powerRB < -1) {powerRB = -1f;}
            if (powerLF > 1) {powerLF = 1f;} else if (powerLF < -1) {powerLF = -1f;}
            if (powerLB > 1) {powerLB = 1f;} else if (powerLB < -1) {powerLB = -1f;}
    
            if (gamepad1.right_trigger > 0.5) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }
    
            lf.setPower(powerLF);
            lb.setPower(powerLB);
            rf.setPower(powerRF);
            rb.setPower(powerRB);
    
            telemetry.addData("rfs", (float)((rf.getCurrentPosition() - rfLastPos) / dt) / 24);
            telemetry.addData("rbs", (float)((rb.getCurrentPosition() - rbLastPos) / dt) / 24);
            telemetry.addData("lfs", (float)((lf.getCurrentPosition() - lfLastPos) / dt) / 24);
            telemetry.addData("lbs", (float)((lb.getCurrentPosition() - lbLastPos) / dt) / 24);
            rfLastPos = rf.getCurrentPosition();
            rbLastPos = rb.getCurrentPosition();
            lfLastPos = lf.getCurrentPosition();
            lbLastPos = lb.getCurrentPosition();
            
        }
        
        setPower(0);
    }
    
    public void setPower (double p) {
        rf.setPower(p);
        rb.setPower(p);
        lf.setPower(p);
        lb.setPower(p);
    }
    
    public void setTargetPosition (int p) {
        rf.setTargetPosition(p);
        rf.setTargetPosition(p);
        rf.setTargetPosition(p);
        rf.setTargetPosition(p);
    }
    
    public void driveInches (double inches) {
        setTargetPosition((int)(countPerRev * (inches / (4 * Math.PI))));
    }
}