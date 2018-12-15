package org.firstinspires.ftc.teamcode.frontier.autonomous.test;

import android.graphics.PointF;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.visuals.Graph;
import org.opencv.core.Scalar;

import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

// Created on 12/10/2018 at 9:03 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.frontier.autonomous.test

@Autonomous(name = "Encoder Test", group = "test")
//@Disabled
public class EncoderTest extends LinearOpMode {
    
    private static final int countPerRev = 1440;
    private static final double inchesDis = 36.0;
    private static final String TAG = "ENCODER_WORK";
    
    private DcMotor rf, rb, lf, lb;
    private BNO055IMU imu;
    
    DecimalFormat df = new DecimalFormat("#.##");
    
    @Override
    public void runOpMode() {
    
        df.setRoundingMode(RoundingMode.HALF_UP);
    
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
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.gyroPowerMode       = BNO055IMU.GyroPowerMode.DEEP;
        parameters.loggingEnabled      = false;
    
        imu = hardwareMap.get(BNO055IMU.class, "imul");
    
        imu.initialize(parameters);
    
        while (!imu.isGyroCalibrated()) {
            telemetry.addData("Status", "Calibrating gyro...");
            telemetry.update();
        }
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        ElapsedTime timer = new ElapsedTime();
        double startangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        double startvolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
        timer.reset();
//        //forwardInches(36, .8);
//        strafeInches(5760, .8, false);
        turnDegrees(45, .8, false);
        double time = timer.seconds();
        double anglechange = getTurnAngle(startangle, false);
        Log.d(TAG, "Done in " + time + " seconds. Angle change was " + anglechange + " degrees. Start volt. was " + startvolt + " volts.");
        
//        while (opModeIsActive()) {
//            ElapsedTime timer = new ElapsedTime();
//            double startangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
//            double startvolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
//            timer.reset();
//            turnDegrees(45, .5, true);
//            double time = timer.seconds();
//            double error = 45-getTurnAngle(startangle, true);
//            Log.d(TAG + "_THIS", "Done in " + time + " seconds. Angle error was " + error + " degrees. Start volt. was " + startvolt + " volts.");
//            sleep(2000);
//        }
    
//        Graph g = new Graph("Speed over 3ft", "Time", "Speed (rpm)", 1200, 720, 0, 6, 1, 0, 100, 10);
//        List<PointF> rfp = new ArrayList<>(), rbp = new ArrayList<>(), lfp = new ArrayList<>(), lbp = new ArrayList<>();
//        ElapsedTime t = new ElapsedTime();
//
//        driveInches(inchesDis);
//        setPower(.8);
//
//        int rfLastPos = 0, rbLastPos = 0, lfLastPos = 0, lbLastPos = 0;
//        double lastTime = 0;
//
//        t.reset();
//        while (t.seconds() <= 6) {
//            double dt = t.seconds() - lastTime;
//            lastTime += dt;
//
//            int
//                    erf = 1120 - rf.getCurrentPosition(),
//                    erb = 1120 - rb.getCurrentPosition(),
//                    elf = 1120 - lf.getCurrentPosition(),
//                    elb = 1120 - lb.getCurrentPosition(),
//                    slow = Math.max(Math.max(erf, erb), Math.max(elf, elb));
//            if (slow != 0 && Math.min(Math.min(erf, erb), Math.min(elf, elb)) >= 0) {
//                rf.setPower(.8 * ((double) erf / slow));
//                rb.setPower(.8 * ((double) erb / slow));
//                lf.setPower(.8 * ((double) elf / slow));
//                lb.setPower(.8 * ((double) elb / slow));
//            }
//
//            rfp.add(new PointF((float)t.seconds(), (float)((rf.getCurrentPosition() - rfLastPos) / dt) / 24));
//            rbp.add(new PointF((float)t.seconds(), (float)((rb.getCurrentPosition() - rbLastPos) / dt) / 24));
//            lfp.add(new PointF((float)t.seconds(), (float)((lf.getCurrentPosition() - lfLastPos) / dt) / 24));
//            lbp.add(new PointF((float)t.seconds(), (float)((lb.getCurrentPosition() - lbLastPos) / dt) / 24));
//            rfLastPos = rf.getCurrentPosition();
//            rbLastPos = rb.getCurrentPosition();
//            lfLastPos = lf.getCurrentPosition();
//            lbLastPos = lb.getCurrentPosition();
//        }
//
//        setPower(0);
//
//        g.plot(rfp, 2, new Scalar(255,0,0));
//        g.plot(rbp, 2, new Scalar(0,255,0));
//        g.plot(lfp, 2, new Scalar(0,0,255));
//        g.plot(lbp, 2, new Scalar(255,145,0));
//
//        g.saveGraph("stock pid speed test at 80 3ft.png");
        
//        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        int rfLastPos = rf.getCurrentPosition(), rbLastPos = rb.getCurrentPosition(), lfLastPos = lf.getCurrentPosition(), lbLastPos = lb.getCurrentPosition();
//        ElapsedTime t = new ElapsedTime();
//
//        t.reset();
//        double lastTime = 0;
//        while (opModeIsActive()) {
//
//            double dt = t.seconds() - lastTime;
//            lastTime = t.seconds();
//
//            float powerRF, powerRB, powerLF, powerLB;
//            float
//                    straight    = -gamepad1.left_stick_y,
//                    strafe      = gamepad1.left_stick_x,
//                    rotate      = gamepad1.right_stick_x;
//            powerRF = straight;
//            powerRB = straight;
//            powerLF = straight;
//            powerLB = straight;
//            powerRF -= strafe;
//            powerRB += strafe;
//            powerLF += strafe;
//            powerLB -= strafe;
//            powerRF -= rotate;
//            powerRB -= rotate;
//            powerLF += rotate;
//            powerLB += rotate;
//            if (powerRF > 1) {powerRF = 1f;} else if (powerRF < -1) {powerRF = -1f;}
//            if (powerRB > 1) {powerRB = 1f;} else if (powerRB < -1) {powerRB = -1f;}
//            if (powerLF > 1) {powerLF = 1f;} else if (powerLF < -1) {powerLF = -1f;}
//            if (powerLB > 1) {powerLB = 1f;} else if (powerLB < -1) {powerLB = -1f;}
//
//            if (gamepad1.right_trigger > 0.5) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }
//
//            lf.setPower(powerLF);
//            lb.setPower(powerLB);
//            rf.setPower(powerRF);
//            rb.setPower(powerRB);
//
//            telemetry.addData("rfs", (float)((rf.getCurrentPosition() - rfLastPos) / dt) / 24);
//            telemetry.addData("rbs", (float)((rb.getCurrentPosition() - rbLastPos) / dt) / 24);
//            telemetry.addData("lfs", (float)((lf.getCurrentPosition() - lfLastPos) / dt) / 24);
//            telemetry.addData("lbs", (float)((lb.getCurrentPosition() - lbLastPos) / dt) / 24);
//            telemetry.update();
//            rfLastPos = rf.getCurrentPosition();
//            rbLastPos = rb.getCurrentPosition();
//            lfLastPos = lf.getCurrentPosition();
//            lbLastPos = lb.getCurrentPosition();
//
//        }
        
        setPower(0);
        
        requestOpModeStop();
    }
    
    public void setPower (double p) {
        rf.setPower(p);
        rb.setPower(p);
        lf.setPower(p);
        lb.setPower(p);
    }
    
    public void setTargetPosition (int p) {
        rf.setTargetPosition(p);
        rb.setTargetPosition(p);
        lf.setTargetPosition(p);
        lb.setTargetPosition(p);
    }
    
    public void driveInches (double inches) {
        setTargetPosition((int)(countPerRev * (inches / (4 * Math.PI))));
    }
    
    public void forwardInches (double inches, double maxPower) {
        int goalPosition = (int)(countPerRev * (inches / (4 * Math.PI)));
        int toSlow = goalPosition / 4;
        
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        rf.setTargetPosition(rf.getCurrentPosition() + goalPosition);
        rb.setTargetPosition(rb.getCurrentPosition() + goalPosition);
        lf.setTargetPosition(lf.getCurrentPosition() + goalPosition);
        lb.setTargetPosition(lb.getCurrentPosition() + goalPosition);
        
        int avgErr = Integer.MAX_VALUE;
        
        while (avgErr > 10) {
            double accelPow;
            
            int
                    erf = Math.abs(rf.getTargetPosition() - rf.getCurrentPosition()),
                    erb = Math.abs(rb.getTargetPosition() - rb.getCurrentPosition()),
                    elf = Math.abs(lf.getTargetPosition() - lf.getCurrentPosition()),
                    elb = Math.abs(lb.getTargetPosition() - lb.getCurrentPosition()),
                    slow = Math.max(Math.max(erf, erb), Math.max(elf, elb));
            
            if (Math.abs(slow) < Math.abs(toSlow)) accelPow = maxPower * (slow/toSlow);
            else accelPow = Math.abs(rf.getCurrentPosition()) > 1440 ? maxPower : Range.clip(Math.pow(Math.E, Math.abs(rf.getCurrentPosition()) / 1440.0) - .9, 0, maxPower);
            
            
            if (slow != 0 /*&& Math.min(Math.min(erf, erb), Math.min(elf, elb)) >= 0*/) {
                rf.setPower(accelPow * ((double) erf / slow));
                lb.setPower(accelPow * ((double) elb / slow));
                lf.setPower(accelPow * ((double) elf / slow));
                rb.setPower(accelPow * ((double) erb / slow));
                
                
            }
            //avgErr = (Math.abs(erf) + Math.abs(erb) + Math.abs(elf) + Math.abs(elb)) / 4;
            avgErr = (erf + erb + elf + elb) / 4;
        }
        
        rf.setPower(0);
        lb.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        
    }
    
    public void strafeInches (double inches, double maxPower, boolean right) {
        inches = Math.abs(inches);
        int goalPosition = (int)(countPerRev * (inches / (4 * Math.PI))), dir = right ? 1 : -1; // TODO: Edit strafe inches to actually coordinate to inches
        int toSlow = goalPosition / 4;
    
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        rf.setTargetPosition(rf.getCurrentPosition() + goalPosition * -dir);
        rb.setTargetPosition(rb.getCurrentPosition() + goalPosition * dir);
        lf.setTargetPosition(lf.getCurrentPosition() + (int)(goalPosition*.9) * dir);
        lb.setTargetPosition(lb.getCurrentPosition() + goalPosition * -dir);
        
        int avgErr = Integer.MAX_VALUE;
        
        while (avgErr > 10) {
            double accelPow;
            
            int
                    erf = Math.abs(rf.getTargetPosition() - rf.getCurrentPosition()),
                    erb = Math.abs(rb.getTargetPosition() - rb.getCurrentPosition()),
                    elf = Math.abs(lf.getTargetPosition() - lf.getCurrentPosition()),
                    elb = Math.abs(lb.getTargetPosition() - lb.getCurrentPosition()),
                    slow = Math.max(Math.max(erf, erb), Math.max(elf, elb));
            
            if (slow < toSlow) accelPow = maxPower * (slow/toSlow);
            else accelPow = Math.abs(rf.getCurrentPosition()) > 1440 ? maxPower : Range.clip(Math.pow(Math.E, Math.abs(rf.getCurrentPosition()) / 1440.0) - .9, 0, maxPower);
            
            
            if (slow != 0 /*&& Math.min(Math.min(erf, erb), Math.min(elf, elb)) >= 0*/) {
                rf.setPower(accelPow * ((double) erf / slow));
                lb.setPower(accelPow * ((double) elb / slow));
                lf.setPower(accelPow * ((double) elf / slow));
                rb.setPower(accelPow * ((double) erb / slow));
                
                
            }
            //avgErr = (Math.abs(erf) + Math.abs(erb) + Math.abs(elf) + Math.abs(elb)) / 4;
            avgErr = (erf + erb + elf + elb) / 4;
        }
        
        rf.setPower(0);
        lb.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        
    }
    public void strafeInches (int goalPosition, double maxPower, boolean right) {
        int dir = right ? 1 : -1; // TODO: Edit strafe inches to actually coordinate to inches
        int toSlow = goalPosition / 4;
        
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        rf.setTargetPosition(rf.getCurrentPosition() + goalPosition * -dir);
        rb.setTargetPosition(rb.getCurrentPosition() + goalPosition * dir);
        lf.setTargetPosition(lf.getCurrentPosition() + (int)(goalPosition*.9) * dir);
        lb.setTargetPosition(lb.getCurrentPosition() + goalPosition * -dir);
        
        int avgErr = Integer.MAX_VALUE;
        
        while (avgErr > 10) {
            double accelPow;
            
            int
                    erf = Math.abs(rf.getTargetPosition() - rf.getCurrentPosition()),
                    erb = Math.abs(rb.getTargetPosition() - rb.getCurrentPosition()),
                    elf = Math.abs(lf.getTargetPosition() - lf.getCurrentPosition()),
                    elb = Math.abs(lb.getTargetPosition() - lb.getCurrentPosition()),
                    slow = Math.max(Math.max(erf, erb), Math.max(elf, elb));
            
            if (slow < toSlow) accelPow = maxPower * (slow/toSlow);
            else accelPow = Math.abs(rf.getCurrentPosition()) > 1440 ? maxPower : Range.clip(Math.pow(Math.E, Math.abs(rf.getCurrentPosition()) / 1440.0) - .9, 0, maxPower);
            
            
            if (slow != 0 /*&& Math.min(Math.min(erf, erb), Math.min(elf, elb)) >= 0*/) {
                rf.setPower(accelPow * ((double) erf / slow));
                lb.setPower(accelPow * ((double) elb / slow));
                lf.setPower(accelPow * ((double) elf / slow));
                rb.setPower(accelPow * ((double) erb / slow));
                
                
            }
            //avgErr = (Math.abs(erf) + Math.abs(erb) + Math.abs(elf) + Math.abs(elb)) / 4;
            avgErr = (erf + erb + elf + elb) / 4;
        }
        
        rf.setPower(0);
        lb.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        
    }
    
    public void turnDegrees (double degrees, double maxPower, boolean clockwise) { // clockwise is + direction for imul
        int dir = clockwise ? 1 : -1;
        degrees = dir*(Math.abs(degrees) + 2 * (Math.abs(degrees) / 100)); // ensure no ones giving me a negative rotation. that's for the clockwise var
    
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        final double
                kp = 6,
                ki = 2.5,//3,
                kd = .02,
                thres = 3,
                toSlow = degrees / 4;
        
        double
                startangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
                e = degrees,
                lastTime = 0,
                laste = e,
                sume = 0;
        ElapsedTime t = new ElapsedTime();
        
        t.reset();
        while ((e > thres || e < -thres) && opModeIsActive()) {
            double dt = t.seconds() - lastTime;
            lastTime += dt;
            
            e = degrees - getTurnAngle(startangle, clockwise);
            double
                    pi = e * dt,
                    p = (kp * e + ki * (pi + sume) + kd * ((e - laste)/dt)) / 100;
    
            double accelPow = maxPower;
            if (Math.abs(e) < Math.abs(toSlow)) accelPow = maxPower * (e/toSlow) + .1;
            //else accelPow = Math.abs(rf.getCurrentPosition()) > 1440 ? maxPower : Range.clip(Math.pow(Math.E, Math.abs(rf.getCurrentPosition()) / 1440.0) - .9, 0, maxPower);

            if (Math.abs(p) > accelPow && e * p >= 0) {
                p = accelPow * dir;
            } else {
                if (Math.abs(p) > accelPow) p = accelPow * dir;
                sume += pi;
            }
            
//            if (Math.abs(p) > maxPower && e * p >= 0) {
//                p = maxPower * Math.signum(e) ;
//            } else {
//                if (Math.abs(p) > maxPower) p = maxPower * Math.signum(e);
//                sume += pi;
//            }
    
            rf.setPower(-p);
            lb.setPower(p);
            lf.setPower(p);
            rb.setPower(-p);
            
            Log.d(TAG, "dt: " + df.format(dt) + "; e: " + df.format(e) + "; p: " + df.format(p) + "; sum: " + df.format(sume));
            
        }
        
        rf.setPower(0);
        lb.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        
    }
    private double getTurnAngle (double start, boolean clockwise) {
        double cur = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        //Log.d("TURN_ANGLE", "sta: " + df.format(start) + "; cur: " + df.format(cur));
        int dir = clockwise ? 1 : -1;
        double dif = cur - start;
        if (dif * dir >= 0) return dif;
        return (cur + 360*dir) - start;
    }
}