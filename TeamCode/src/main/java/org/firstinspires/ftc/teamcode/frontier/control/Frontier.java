package org.firstinspires.ftc.teamcode.frontier.control;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.new_frontier.control.ToggleServo;
import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;

import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.Enums.FieldPosition;
import static org.firstinspires.ftc.teamcode.Enums.SampleArrangement;

public class Frontier {
    
    /** CONSTANTS */
    private static final String
            TAG     = "Frontier",
            RFM_N   = "rf",
            RBM_N   = "rb",
            LFM_N   = "lf",
            LBM_N   = "lb",
            LIFTM_N = "l",
            LIFTS_N = "lock",
            MARKLS_N = "markl",
            MARKRS_N = "markr";
    
    private static final double
            UNLOCK_P                = .85,
            LOCK_P                  = .95,
            MARKH_P                 = .4,
            MARKD_P                 = 0.0,
            MARK_LEFT_HOLD          = .25,
            MARK_LEFT_DROP          = .5,
            MARK_RIGHT_HOLD         = .16,
            MARK_RIGHT_DROP         = .4,
            TURN_KP                 = 6,
            TURN_KI                 = 2.5,
            TURN_KD                 = 0.02,
            STRAFE_INCHES_PER_REV   = 8.75;
    
    private static final int
            ENCODER_PER_REV = 560;
    
    private static DecimalFormat DECIMAL_FORMAT = new DecimalFormat("#.##");
    
    /** INSTANCE VARS */
    private OpMode opmode;
    private DcMotor rf, rb, lf, lb;
    private Servo markl, markr;
    private Latcher     latcher;
    private ToggleServo lock;
    
    private double powerRF, powerRB, powerLF, powerLB;
    
    private LinearOpMode lopmode = null;
    private BNO055IMU imu;
    private FieldPosition position = FieldPosition.ERROR;
    private SampleArrangement arrangement = SampleArrangement.ERROR;
    
    /** INITIALIZATION */
    public Frontier (OpMode om, FieldPosition pos) {
        
        opmode = om;
        if (opmode instanceof LinearOpMode) lopmode = (LinearOpMode)opmode;
        init();
        initAuto(pos);
    
    }
    
    private void init () {
    
        rf = opmode.hardwareMap.dcMotor.get(RFM_N);
        rb = opmode.hardwareMap.dcMotor.get(RBM_N);
        lf = opmode.hardwareMap.dcMotor.get(LFM_N);
        lb = opmode.hardwareMap.dcMotor.get(LBM_N);
    
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
    
        latcher = new Latcher(opmode.hardwareMap.dcMotor.get(LIFTM_N));
    
        lock = new ToggleServo(opmode.hardwareMap.servo.get(LIFTS_N), UNLOCK_P, LOCK_P);
        
        markl = opmode.hardwareMap.servo.get(MARKLS_N);
        markr = opmode.hardwareMap.servo.get(MARKRS_N);
        
    }
    
    private void initAuto (FieldPosition pos) {
        if (pos == FieldPosition.ERROR) return;
        if (lopmode == null) throw new RuntimeException("No linear opmode found.");
        position = pos;
    
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
    
        imu = opmode.hardwareMap.get(BNO055IMU.class, "imul");
        imu.initialize(parameters);
    
        VisualsHandler.tempSetup(opmode);
        VisualsHandler.phoneLightOn();
    }
    
    public void waitForStart () {
        ElapsedTime t = new ElapsedTime();
        
        t.reset();
        while (!imu.isGyroCalibrated()) {
            if ((int)t.seconds() % 3 == 0) {
                if (position != FieldPosition.DEPOT_NO_SAMPLE) arrangement = VisualsHandler.takePreviewAndSample();
                debug("Arrangement read as " + arrangement);
                tele("Status", "Calibrating gyro...");
                telewithup("Arrangement", arrangement);
            }
        }
        
        while (!lopmode.isStarted()) {
            if ((int)t.seconds() % 3 == 0) {
                if (position != FieldPosition.DEPOT_NO_SAMPLE) arrangement = VisualsHandler.takePreviewAndSample();
                debug("Arrangement read as " + arrangement);
                tele("Status", "Initialized.");
                telewithup("Arrangement", arrangement);
            }
        }
    }
    
    /** TELEOP */
    public void start () {
    
        lock.open();
        markl.setPosition(MARK_LEFT_HOLD);
        markr.setPosition(MARK_RIGHT_HOLD);
        
    }
    
    public void drive () {
    
        float
                straight    = -opmode.gamepad1.right_stick_y,
                strafe      = opmode.gamepad1.right_stick_x,
                rotate      = opmode.gamepad1.left_stick_x;
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
    
        if (opmode.gamepad1.right_trigger > 0.5) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }
    
        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);
        
        lock.tob(opmode.gamepad1.x);
        
        latcher.run(
                opmode.gamepad1.dpad_up,
                opmode.gamepad1.dpad_down,
                opmode.gamepad1.left_bumper,
                opmode.gamepad1.left_trigger > .5,
                opmode.gamepad1.right_stick_button && opmode.gamepad1.left_stick_button);
        
    }
    
    public void stop () {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        latcher.stop();
    }
    
    /** AUTONOMOUS */
    public void auto () throws InterruptedException {
        if (!lopmode.opModeIsActive()) return;
        
        // ensure that we know the sample arrangement
        VisualsHandler.Sampler sampler = VisualsHandler.offloadPreviewandSample();
        Thread t = new Thread(sampler);
        t.start();
        
        // drop down
        latcher.raise();
        
        t.join();
        SampleArrangement temp = sampler.getValue();
        if (temp != SampleArrangement.ERROR) arrangement = temp;
        debug("Sample arrangement is: " + arrangement);
        
        latcher.waitFor(lopmode);
        
        // drive backwards 12in
        //if (arrangement == SampleArrangement.MIDDLE && position != FieldPosition.DEPOT_NO_SAMPLE) forwardInches(-28, 1);
        /*else*/
        if (position == FieldPosition.DEPOT_NO_SAMPLE) forwardInches(-27, .5);
        else forwardInches(-17, .5);
        
        latcher.lower(); // lower latcher back down for teleop
        
        latcher.waitFor(lopmode);
        
//        double correction = 26; // inches to strafe to clear the sampling field
//
//        // strafe according to arragement
//        if (position != FieldPosition.DEPOT_NO_SAMPLE) {
//            switch (arrangement) {
//                case LEFT:
//                    strafeInches(16, 1, true);
//                    correction -= 16;
//                    break;
//                case RIGHT:
//                    strafeInches(10, 1, false);
//                    correction += 10;
//            }
//
//            if (arrangement != SampleArrangement.ERROR) {
//                // back up 20in
//                if (arrangement != SampleArrangement.MIDDLE) forwardInches(-11, 1);
//
//                // drive back forward 14in
//                forwardInches(11, 1);
//            }
//        }
//
//        // clear the sampling field (by driving 26in right of center)
//        debug("Correction is: " + correction);
//        strafeInches(correction, 1, true);
//
//        // rotate to be flat against the wall
//        if (position == FieldPosition.DEPOT || position == FieldPosition.DEPOT_NO_SAMPLE) {
//            // turn flat to wall
//            turnDegrees(45, .5, false);
//
//            // back up to wall
//            forwardInches(-14, 1);
//
//            // strafe to depot
//            strafeInches(44, 1, false);
//        }
//        else if (position == FieldPosition.CRATER) {
//
//            turnDegrees(45, .5, true);
//
//            strafeInches(10, 1, true);
//
//            forwardInches(52, 1);
//
//            turnDegrees(90, .8, true);
//        }
//
//
//        // drop marker
//        //markl.setPosition(MARKD_P);
//        markl.setPosition(MARK_LEFT_DROP);
//        markr.setPosition(MARK_RIGHT_DROP);
//
////        if (position == FieldPosition.CRATER) {
////            turnDegrees(90, .8, true);
////            forwardInches(54, 1);
////        } else
//
//        // drive and park in crater
//        if (position == FieldPosition.CRATER) {
//            strafeInches(56, 1, true);
//        } else
//        strafeInches(68, 1, true);
//
//        // reset robot for teleop
//        markl.setPosition(MARK_LEFT_HOLD);
//        markr.setPosition(MARK_RIGHT_HOLD);
        
        VisualsHandler.tempSetupStop();
    
    }
    
    public void forwardInches (double inches, double maxPower) {
        if (!lopmode.opModeIsActive()) return;
        int goalPosition = (int)(ENCODER_PER_REV * (inches / (4 * Math.PI)));
        debug("Goal position: " + goalPosition);
        int toSlow = 1000;
        ElapsedTime t = new ElapsedTime();
        
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
        
        t.reset();
        while (avgErr > 10 && lopmode.opModeIsActive()) {
            double accelPow;
            
            int
                    erf = Math.abs(rf.getTargetPosition() - rf.getCurrentPosition()),
                    erb = Math.abs(rb.getTargetPosition() - rb.getCurrentPosition()),
                    elf = Math.abs(lf.getTargetPosition() - lf.getCurrentPosition()),
                    elb = Math.abs(lb.getTargetPosition() - lb.getCurrentPosition()),
                    slow = Math.max(Math.max(erf, erb), Math.max(elf, elb));
            
            if (Math.abs(slow) < Math.abs(toSlow)) accelPow = maxPower * ((double)slow/toSlow);
            else accelPow = Math.abs(Math.abs(goalPosition) - avgErr) > 1440 ? maxPower : Range.clip(Math.pow(Math.E, Math.abs(Math.abs(goalPosition) - avgErr) / 1440.0) - .9, 0, maxPower);
            
            debug("Accel power: " + accelPow + "; Avg error: " + avgErr + "; Slow: " + slow);
            
            if (slow != 0 /*&& Math.min(Math.min(erf, erb), Math.min(elf, elb)) >= 0*/) {
                rf.setPower(accelPow * ((double) erf / slow));
                lb.setPower(accelPow * ((double) elb / slow));
                lf.setPower(accelPow * ((double) elf / slow));
                rb.setPower(accelPow * ((double) erb / slow));
            }
            //avgErr = (Math.abs(erf) + Math.abs(erb) + Math.abs(elf) + Math.abs(elb)) / 4;
            avgErr = (erf + erb + elf + elb) / 4;
        }
    
        debug("Drove " + DECIMAL_FORMAT.format(((double)(goalPosition - avgErr) / ENCODER_PER_REV) * (4 * Math.PI)) + " inches in " + DECIMAL_FORMAT.format(t.seconds()) + " seconds.");
        
        rf.setPower(0);
        lb.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        
    }
    
    public void strafeInches (double inches, double maxPower, boolean right) {
        if (!lopmode.opModeIsActive()) return;
        inches = Math.abs(inches);
        int goalPosition = (int)(ENCODER_PER_REV * (inches / STRAFE_INCHES_PER_REV)), dir = right ? 1 : -1;
        int toSlow = 1000;
        ElapsedTime t = new ElapsedTime();
    
        if (rf.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        
        rf.setTargetPosition(rf.getCurrentPosition() + goalPosition * -dir);
        rb.setTargetPosition(rb.getCurrentPosition() + goalPosition * dir);
        lf.setTargetPosition(lf.getCurrentPosition() + (int)(goalPosition*.9) * dir);
        lb.setTargetPosition(lb.getCurrentPosition() + goalPosition * -dir);
        
        int avgErr = Math.abs(goalPosition);
        
        t.reset();
        while (avgErr > 10 && lopmode.opModeIsActive()) {
            double accelPow;
            
            int
                    erf = Math.abs(rf.getTargetPosition() - rf.getCurrentPosition()),
                    erb = Math.abs(rb.getTargetPosition() - rb.getCurrentPosition()),
                    elf = Math.abs(lf.getTargetPosition() - lf.getCurrentPosition()),
                    elb = Math.abs(lb.getTargetPosition() - lb.getCurrentPosition()),
                    slow = Math.max(Math.max(erf, erb), Math.max(elf, elb));
            
            if (slow < toSlow) accelPow = maxPower * ((double)slow/toSlow);
            else accelPow = Math.abs(Math.abs(goalPosition) - avgErr) > 1440 ? maxPower : Range.clip(Math.pow(Math.E, Math.abs(Math.abs(goalPosition) - avgErr) / 1440.0) - .9, 0, maxPower);
            
            
            if (slow != 0 /*&& Math.min(Math.min(erf, erb), Math.min(elf, elb)) >= 0*/) {
                rf.setPower(accelPow * ((double) erf / slow));
                lb.setPower(accelPow * ((double) elb / slow));
                lf.setPower(accelPow * ((double) elf / slow));
                rb.setPower(accelPow * ((double) erb / slow));
            }
            //avgErr = (Math.abs(erf) + Math.abs(erb) + Math.abs(elf) + Math.abs(elb)) / 4;
            avgErr = (erf + erb + elf + elb) / 4;
        }
    
        debug("Strafed " + DECIMAL_FORMAT.format(((double)(goalPosition - avgErr) / ENCODER_PER_REV) * STRAFE_INCHES_PER_REV ) + " inches in " + DECIMAL_FORMAT.format(t.seconds()) + " seconds.");
        
        rf.setPower(0);
        lb.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        
    }
    
    public void turnDegrees (double degrees, double maxPower, boolean clockwise) { // clockwise is + direction for imul
        if (!lopmode.opModeIsActive()) return;
        int dir = clockwise ? 1 : -1;
        degrees = dir*(Math.abs(degrees) + 2 * (Math.abs(degrees) / 100)); // ensure no ones giving me a negative rotation. that's for the clockwise var
        
        if (rf.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
        final double
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
        while ((e > thres || e < -thres) && lopmode.opModeIsActive()) {
            double dt = t.seconds() - lastTime;
            lastTime += dt;
            
            e = degrees - getAngle(startangle, clockwise);
            double
                    pi = e * dt,
                    p = (TURN_KP * e + TURN_KI * (pi + sume) + TURN_KD * ((e - laste)/dt)) / 100,
                    accelPow = maxPower;
            
            if (Math.abs(e) < Math.abs(toSlow)) accelPow = maxPower * (e/toSlow) + .1;
            
            if (Math.abs(p) > accelPow && e * p >= 0) p = accelPow * dir;
            else {
                if (Math.abs(p) > accelPow) p = accelPow * dir;
                sume += pi;
            }
            
            rf.setPower(-p);
            lb.setPower(p);
            lf.setPower(p);
            rb.setPower(-p);
            
            debug("dt: " + DECIMAL_FORMAT.format(dt) + "; e: " + DECIMAL_FORMAT.format(e) + "; p: " + DECIMAL_FORMAT.format(p) + "; sum: " + DECIMAL_FORMAT.format(sume));
            
        }
        
        debug("Turned " + DECIMAL_FORMAT.format(getAngle(startangle, clockwise)) + " degrees in " + DECIMAL_FORMAT.format(t.seconds()) + " seconds.");
        
        rf.setPower(0);
        lb.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        
    }
    private double getAngle (double start, boolean clockwise) {
        double cur = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        debug("Cur: " + cur + "; Start: " + start);
        //Log.d("TURN_ANGLE", "sta: " + DECIMAL_FORMAT.format(start) + "; cur: " + DECIMAL_FORMAT.format(cur));
        int dir = clockwise ? 1 : -1;
        double dif = cur - start;
        if (dif * dir >= 0) return dif;
        return (cur + 360*dir) - start;
    }
    
    /** DEBUGGING */
    private void tele (String caption, Object data) { opmode.telemetry.addData(caption, data); }
    private void teleup () { opmode.telemetry.update(); }
    private void telewithup (String caption, Object data) {opmode.telemetry.addData(caption, data); opmode.telemetry.update(); }
    
    private static void debug (String message) { Log.d(TAG, message); }
    private static void warn  (String message) { Log.w(TAG, message); }
    private static void error (String message) { Log.e(TAG, message); }
    
}
