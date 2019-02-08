package org.firstinspires.ftc.teamcode.new_frontier.autonomous.test;

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

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

// Created on 1/28/2019 at 9:32 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.new_frontier.autonomous.test

@Autonomous(name = "AccelTest", group = "test")
//@Disabled
public class AccelTest extends LinearOpMode {
    
    private DcMotor rf, rb, lf, lb;
    private BNO055IMU imu;
    //final private int secToMax = 1, secToMin = 1;
    private static final double minPower = .1, percentPerSecond = .7, STRAFE_INCHES_PER_REV   = 8.75;
    private static double
    TURN_KP                 = .01,
    TURN_KI                 = .002,
    TURN_KD                 = 0;
    private static final int ENCODER_PER_REV = 560, slow_down_start = 800;
    private double maxPower = .6;
    private double increment = .005;
    private boolean
            upd = false,
            downd = false,
            rbd = false,
            lbd = false,
            xd = false,
            yd = false;
    private int sel = 1;
    
    private static DecimalFormat DECIMAL_FORMAT = new DecimalFormat("#.##");
    
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
    
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
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
    
        
        // COUNTERCLOCKWISE IS POSITIVE
//
//        //straightInches(36, .5);
//
//        //sleep(1000);
//
//        //straightInches(-36, .5);
//        //straightInches(24, .5);
//        //straightInches(12, .5);
//        strafeInches(24, .5, true);
//
        
        //turnDegrees(90, .5, true);
        
        //double end = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        
        
        double error = 0;
        
        
        while (opModeIsActive()) {
    
            double start = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            
            while (!gamepad1.a && opModeIsActive()) {
                
                if (!upd && gamepad1.dpad_up) {
                    switch (sel) {
                        case 1:
                            TURN_KP += increment;
                            break;
                        case 2:
                            TURN_KI += increment;
                            break;
                        case 3:
                            TURN_KD += increment;
                            break;
                    }
                    upd = true;
                } else if (upd && !gamepad1.dpad_up) upd = false;
                
                if (!downd && gamepad1.dpad_down) {
                    switch (sel) {
                        case 1:
                            TURN_KP -= increment;
                            break;
                        case 2:
                            TURN_KI -= increment;
                            break;
                        case 3:
                            TURN_KD -= increment;
                            break;
                    }
                    downd = true;
                } else if (downd && !gamepad1.dpad_down) downd = false;
                
                if (!rbd && gamepad1.right_bumper) {
                    sel++;
                    if (sel == 4) sel = 1;
                    rbd = true;
                } else if (rbd && !gamepad1.right_bumper) rbd = false;
                
                if (!lbd && gamepad1.left_bumper) {
                    sel--;
                    if (sel == 0) sel = 3;
                    lbd = true;
                } else if (lbd && !gamepad1.left_bumper) lbd = false;
                
                if (!yd && gamepad1.y) {
                    increment *= 10;
                    yd = true;
                } else if (yd && !gamepad1.y) yd = false;
                
                if (!xd && gamepad1.x) {
                    increment *= .1;
                    xd = true;
                } else if (xd && !gamepad1.x) xd = false;
                
                switch (sel) {
                    case 1:
                        telemetry.addData("KP sel", TURN_KP);
                        telemetry.addData("KI not", TURN_KI);
                        telemetry.addData("KD not", TURN_KD);
                        break;
                    case 2:
                        telemetry.addData("KP not", TURN_KP);
                        telemetry.addData("KI sel", TURN_KI);
                        telemetry.addData("KD not", TURN_KD);
                        break;
                    case 3:
                        telemetry.addData("KP not", TURN_KP);
                        telemetry.addData("KI not", TURN_KI);
                        telemetry.addData("KD sel", TURN_KD);
                        break;
                }
                telemetry.addData("Increment", increment);
                telemetry.addData("Last error", error);
                telemetry.update();
            }
            turnDegrees(90, .5, true);
            error = deltaTheta(start, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
            
            //drive();
    
            //telemetry.addData("error", end - start);
//            telemetry.addData("Start: ", start);
//            telemetry.addData("Delta: ", deltaTheta(start, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle));
//            telemetry.addData("Status", "Done.");
            //telemetry.update();
            
        }
    }
    
    private void drive () {
        if (gamepad1.right_bumper) maxPower = 1;
        else maxPower = .6;
    
        double powerRF, powerRB, powerLF, powerLB;
        double
                straight    = -gamepad1.right_stick_y,
                strafe      = gamepad1.right_stick_x,
                rotate      = gamepad1.left_stick_x;
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
    
        lf.setPower(powerLF*maxPower);
        lb.setPower(powerLB*maxPower);
        rf.setPower(powerRF*maxPower);
        rb.setPower(powerRB*maxPower);
    }
    
    private double deltaTheta (double start, double current) {
        double ret = current - start;
        //debug(DECIMAL_FORMAT.format(start) + ", " + DECIMAL_FORMAT.format(current) + ", " + DECIMAL_FORMAT.format(ret));
        if (ret < -180) ret += 360;
        else if (ret > 180) ret -= 360;
        return ret;
    }
    
    private void keepAngle (double maxPower) {
    
    }
    
    private double currentPower = 0;
    private void straightInches (double inches, double maxPower) {
    
        int goalPosition = (int)(ENCODER_PER_REV * (inches / (4 * Math.PI)));
        debug("Goal position: " + goalPosition);
        ElapsedTime t      = new ElapsedTime();
    
        if (rf.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    
        rf.setTargetPosition(rf.getCurrentPosition() + goalPosition);
        rb.setTargetPosition(rb.getCurrentPosition() + goalPosition);
        lf.setTargetPosition(lf.getCurrentPosition() + goalPosition);
        lb.setTargetPosition(lb.getCurrentPosition() + goalPosition);
    
        int avgErr = Math.abs(goalPosition);
        double lastTime = 0;
        boolean _short = Math.abs(goalPosition) < slow_down_start;
    
        t.reset();
        while (avgErr > 10 && opModeIsActive()) {
            double dt = t.seconds() - lastTime;
            lastTime += dt;
        
            int
                    erf = Math.abs(rf.getTargetPosition() - rf.getCurrentPosition()),
                    erb = Math.abs(rb.getTargetPosition() - rb.getCurrentPosition()),
                    elf = Math.abs(lf.getTargetPosition() - lf.getCurrentPosition()),
                    elb = Math.abs(lb.getTargetPosition() - lb.getCurrentPosition()),
                    slow = Math.max(Math.max(erf, erb), Math.max(elf, elb));
        
            //if (Math.abs(slow) < Math.abs(toSlow)) accelPow = maxPower * ((double)slow/toSlow);
            //else accelPow = Math.abs(Math.abs(goalPosition) - avgErr) > 1440 ? maxPower : Range.clip(Math.pow(Math.E, Math.abs(Math.abs(goalPosition) - avgErr) / 1440.0) - .9, 0, maxPower);
        
            if (!_short) {
                if (currentPower < maxPower && slow >= slow_down_start) {
                    currentPower += dt * percentPerSecond;
                    if (currentPower > maxPower) currentPower = maxPower;
                } else if (slow < slow_down_start && currentPower > minPower) {
                    currentPower -= dt * percentPerSecond;
                    if (currentPower < minPower) currentPower = minPower;
                }
            } else {
                if (Math.abs((double) slow / goalPosition) > .75) {
                    currentPower += dt * percentPerSecond;
                    if (currentPower > maxPower) currentPower = maxPower;
                } else {
                    currentPower -= dt * percentPerSecond;
                    if (currentPower < minPower) currentPower = minPower;
                }
            }
            
            debug("dt: " + DECIMAL_FORMAT.format(dt) + "; Current power: " + DECIMAL_FORMAT.format(currentPower) + "; Avg error: " + avgErr + "; Slow error: " + slow);
        
            if (slow != 0 /*&& Math.min(Math.min(erf, erb), Math.min(elf, elb)) >= 0*/) {
                rf.setPower(currentPower * ((double) erf / slow));
                lb.setPower(currentPower * ((double) elb / slow));
                lf.setPower(currentPower * ((double) elf / slow));
                rb.setPower(currentPower * ((double) erb / slow));
            }
            //avgErr = (Math.abs(erf) + Math.abs(erb) + Math.abs(elf) + Math.abs(elb)) / 4;
            avgErr = (erf + erb + elf + elb) / 4;
        }
    
        debug("Drove " + DECIMAL_FORMAT.format(((double)(goalPosition - avgErr) / ENCODER_PER_REV) * (4 * Math.PI)) + " inches in " + DECIMAL_FORMAT.format(t.seconds()) + " seconds.");
    
        rf.setPower(0);
        lb.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        
        currentPower = 0;
        
    }
    private void strafeInches (double inches, double maxPower, boolean right) {
        inches = Math.abs(inches);
        int goalPosition = (int)(ENCODER_PER_REV * (inches / STRAFE_INCHES_PER_REV)), dir = right ? 1 : -1;
        ElapsedTime t = new ElapsedTime();
    
        if (rf.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    
        rf.setTargetPosition(rf.getCurrentPosition() + goalPosition * -dir);
        rb.setTargetPosition(rb.getCurrentPosition() + goalPosition * dir);
        lf.setTargetPosition(lf.getCurrentPosition() + goalPosition * dir);//(int)(goalPosition*.9) * dir);
        lb.setTargetPosition(lb.getCurrentPosition() + goalPosition * -dir);
    
        int avgErr = Math.abs(goalPosition);
        double lastTime = 0;
        boolean _short = Math.abs(goalPosition) < slow_down_start;
    
//        Graph        g   = new Graph("Position w/ new encoder stuff", "Time", "Position", 1200, 720, 0, 2.5, .5, 0, 1700, 250);
//        List<PointF> rfp = new ArrayList<>(), rbp = new ArrayList<>(), lfp = new ArrayList<>(), lbp = new ArrayList<>();
    
        double dt;
        int erf, erb, elf, elb, slow;
        
        currentPower = maxPower;
        
        t.reset();
        while (avgErr > 10 && opModeIsActive()) {
            //double accelPow;
            dt = t.seconds() - lastTime;
            lastTime += dt;
            
            erf = Math.abs(rf.getTargetPosition() - rf.getCurrentPosition());
            erb = Math.abs(rb.getTargetPosition() - rb.getCurrentPosition());
            elf = Math.abs(lf.getTargetPosition() - lf.getCurrentPosition());
            elb = Math.abs(lb.getTargetPosition() - lb.getCurrentPosition());
            slow = Math.max(Math.max(erf, erb), Math.max(elf, elb));
        
            //if (slow < toSlow) accelPow = maxPower * ((double)slow/toSlow);
            //else accelPow = Math.abs(Math.abs(goalPosition) - avgErr) > 1440 ? maxPower : Range.clip(Math.pow(Math.E, Math.abs(Math.abs(goalPosition) - avgErr) / 1440.0) - .9, 0, maxPower);
    
//            if (!_short) {
//                if (currentPower < maxPower && slow >= slow_down_start) {
//                    currentPower += dt * percentPerSecond;
//                    if (currentPower > maxPower) currentPower = maxPower;
//                } else if (slow < slow_down_start && currentPower > minPower) {
//                    currentPower -= dt * percentPerSecond;
//                    if (currentPower < minPower) currentPower = minPower;
//                }
//            } else {
//                if (Math.abs((double) slow / goalPosition) > .75) {
//                    currentPower += dt * percentPerSecond;
//                    if (currentPower > maxPower) currentPower = maxPower;
//                } else {
//                    currentPower -= dt * percentPerSecond;
//                    if (currentPower < minPower) currentPower = minPower;
//                }
//            }
    
            debug("dt: " + DECIMAL_FORMAT.format(dt) + "; Current power: " + DECIMAL_FORMAT.format(currentPower) + "; Avg error: " + avgErr + "; Slow error: " + slow);
    
            if (slow != 0 /*&& Math.min(Math.min(erf, erb), Math.min(elf, elb)) >= 0*/) {
                rf.setPower(currentPower); //* ((double) erf / slow));
                lb.setPower(currentPower); //* ((double) elb / slow));
                lf.setPower(currentPower); //* ((double) elf / slow));
                rb.setPower(currentPower); //* ((double) erb / slow));
                debug("rf: " + rf.getCurrentPosition() + ", " + rf.getPower());
                debug("rb: " + rb.getCurrentPosition() + ", " + rb.getPower());
                debug("lf: " + lf.getCurrentPosition() + ", " + lf.getPower());
                debug("lb: " + lb.getCurrentPosition() + ", " + lb.getPower());
            }
//            rfp.add(new PointF((float)lastTime, Math.abs(rf.getCurrentPosition())));
//            rbp.add(new PointF((float)lastTime, Math.abs(rb.getCurrentPosition())));
//            lfp.add(new PointF((float)lastTime, Math.abs(lf.getCurrentPosition())));
//            lbp.add(new PointF((float)lastTime, Math.abs(lb.getCurrentPosition())));
            //avgErr = (Math.abs(erf) + Math.abs(erb) + Math.abs(elf) + Math.abs(elb)) / 4;
            avgErr = (erf + erb + elf + elb) / 4;
        }
    
        debug("Strafed " + DECIMAL_FORMAT.format(((double)(goalPosition - avgErr) / ENCODER_PER_REV) * STRAFE_INCHES_PER_REV ) + " inches in " + DECIMAL_FORMAT.format(t.seconds()) + " seconds.");
    
        rf.setPower(0);
        lb.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
    
//        g.plot(rfp, 2, new Scalar(255, 0, 0));
//        g.plot(rbp, 2, new Scalar(0,255,0));
//        g.plot(lfp, 2, new Scalar(0,0,255));
//        g.plot(lbp, 2, new Scalar(255,145,0));
//
//        g.saveGraph("new encoder strafe 2ft .5.png");
        
        currentPower = 0;
    }
    private void turnDegrees (double degrees, double maxPower, boolean clockwise) {
        if (!opModeIsActive()) return;
        int dir = clockwise ? -1 : 1;
        //degrees = dir*(Math.abs(degrees) + 2 * (Math.abs(degrees) / 100)); // ensure no ones giving me a negative rotation. that's for the clockwise var
        degrees = dir*Math.abs(degrees);
        
        if (rf.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    
        final double
                thres = 1,
                toSlow = degrees / 4;
        
        
    
        double
                startangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
                e = degrees,
                lastTime = 0,
                laste = e,
                sume = 0;
        ElapsedTime t = new ElapsedTime();
    
        t.reset();
        while (/*Math.abs(e) > thres &&*/ opModeIsActive() && !gamepad1.b) {
            double dt = t.seconds() - lastTime;
            lastTime += dt;
        
            //e = degrees - getAngle(startangle, clockwise);
            e = degrees - deltaTheta(startangle, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
            //telemetry.addData("theta", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
            //telemetry.update();
//            double
//                    pi = e * dt,
//                    p = (TURN_KP * e + TURN_KI * (pi + sume) + TURN_KD * ((e - laste)/dt)) / 100,
//                    accelPow = maxPower;
        
            double p = TURN_KP * e + TURN_KD * ((e - laste)/dt);
            
            laste = e;
            
            //if (Math.abs(e) < Math.abs(toSlow)) accelPow = maxPower * (e/toSlow) + .1;
        
//            if (Math.abs(p) > accelPow && e * p >= 0) p = accelPow * dir;
//            else {
//                if (Math.abs(p) > accelPow) p = accelPow * dir;
//                sume += pi;
//            }
    
            //debug("dt: " + DECIMAL_FORMAT.format(dt) + "; error: " + DECIMAL_FORMAT.format(e) + "; p: " + DECIMAL_FORMAT.format(p)); //+ "; sum: " + DECIMAL_FORMAT.format(sume));
            debug("dt: " + dt + "; error: " + e + "; p: " + p); //+ "; sum: " + DECIMAL_FORMAT.format(sume));
            
            //if (Math.abs(p) > maxPower) p = maxPower * dir;
            //p = Range.clip(p, -maxPower, maxPower);
            if (p > maxPower) p = maxPower;
            else if (p < -maxPower) p = -maxPower;
        
            rf.setPower(p);
            lb.setPower(-p);
            lf.setPower(-p);
            rb.setPower(p);
        
        }
    
        //debug("Turned " + DECIMAL_FORMAT.format(getAngle(startangle, clockwise)) + " degrees in " + DECIMAL_FORMAT.format(t.seconds()) + " seconds.");
    
        rf.setPower(0);
        lb.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
    }
    
    private static void debug (String message) { Log.d("ACCEL_TEST", message); }
}