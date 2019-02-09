package org.firstinspires.ftc.teamcode.new_frontier.control;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Enums.FieldPosition;
import org.firstinspires.ftc.teamcode.Enums.SampleArrangement;
import org.firstinspires.ftc.teamcode.frontier.control.Latcher;
import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.text.DecimalFormat;

public class NewFrontier {
    
    /** CONSTANTS */
    private static final String
            TAG             = "New Frontier",
            HOMING_SAVE_DIR = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS).toString(),
            HOMING_FILE_N   = "homing.txt",
            RFM_N           = "rf",
            RBM_N           = "rb",
            LFM_N           = "lf",
            LBM_N           = "lb",
            LIFTM_N         = "l",
            LIFTS_N         = "lock",
            MARKLS_N        = "markl",
            MARKRS_N        = "markr",
            SHOULDER_N      = "shoulder",
            ELBOW_N         = "elbow",
            SHOULDERLIM_N   = "slim",
            ELBOWLIM_N      = "elim",
            ROLLER_N        = "roller",
            COLOR_N         = "color",
            IMU_N           = "imul";
    
    private static final double
            MAX_DRIVE_POWER    = .6,
            UNLOCK_P           = .62,
            LOCK_P             = .8,
            MARK_LEFT_HOLD     = .18,
            MARK_LEFT_DROP     = .9,
            MARK_RIGHT_HOLD    = .15,
            MARK_RIGHT_DROP    = .4,
            MIN_POWER          = .1,
            PERCENT_PER_SECOND = .7,
            TURN_KP                 = .06,
            TURN_KI                 = .012,
            TURN_KD                 = 0,
            STRAFE_INCHES_PER_REV   = 8.75;
    
    private static final int
            ENCODER_PER_REV = 560,
            SLOW_DOWN_START = 800;
    
    private static DecimalFormat DECIMAL_FORMAT = new DecimalFormat("#.##");
    
    /** INSTANCE VARS */
    private LinearOpMode opmode;
    private DcMotor rf, rb, lf, lb;
    private ToggleServo markl, markr, lock;
    private Latcher latcher;
    private Arm arm;
    private SingleRoller roller;
    
    private double powerRF, powerRB, powerLF, powerLB, currentMax;
    private boolean locked = false;
    
    private BNO055IMU         imu;
    private FieldPosition     position;
    private SampleArrangement arrangement = SampleArrangement.ERROR;
    private Thread rollerMon;
    
    /** INITIALIZATION */
    public NewFrontier (LinearOpMode om, FieldPosition position) {
        opmode = om;
        this.position = position;
        initAuto();
        init();
    }
    
    public void init () {
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
        
        markl = new ToggleServo(opmode.hardwareMap.servo.get(MARKLS_N), MARK_LEFT_DROP, MARK_LEFT_HOLD);
        markr = new ToggleServo(opmode.hardwareMap.servo.get(MARKRS_N), MARK_RIGHT_DROP, MARK_RIGHT_HOLD);
    
        roller = new SingleRoller(
                opmode.hardwareMap.crservo.get(ROLLER_N),
                (ModernRoboticsI2cColorSensor) opmode.hardwareMap.colorSensor.get(COLOR_N));
    
        final File f = new File(HOMING_SAVE_DIR, HOMING_FILE_N);
        
        if (position != FieldPosition.ERROR || !f.exists()) {
            telewithup("Status", "Homing arm...");
            arm = new Arm(
                    opmode.hardwareMap.dcMotor.get(SHOULDER_N),
                    opmode.hardwareMap.dcMotor.get(ELBOW_N),
                    (ModernRoboticsTouchSensor) opmode.hardwareMap.touchSensor.get(SHOULDERLIM_N),
                    (ModernRoboticsTouchSensor) opmode.hardwareMap.touchSensor.get(ELBOWLIM_N),
                    opmode);
    
            int[] home = arm.init();
            if (f.exists() && !f.delete()) throw new Error("Unable to delete file.");
            try {
                if (!f.createNewFile()) throw new Error("Welp, couldn't make file.");
            } catch (IOException e) { throw new Error("Welp, couldn't make file -- IOException."); }
            try {
                FileOutputStream out  = new FileOutputStream(f);
                PrintWriter      pw = new PrintWriter(out);
                //Enumeration      e  = save.elements();
        
                for (int i : home) pw.println(i);
                pw.flush();
                pw.close();
                out.close();
            } catch (FileNotFoundException e) {
                throw new Error("Welp, couldn't make file -- FileNotFoundException.");
            } catch (IOException e) {
                throw new Error("Welp, couldn't make file -- IOException.");
            }
        } else {
            int[] home = new int[2];
            try {
                FileInputStream in      = new FileInputStream(f);
                BufferedReader  reader = new BufferedReader(new InputStreamReader(in));
        
                String line = reader.readLine();
                for (int i = 0; i < home.length && line != null; i++) {
                    home[i] = Integer.parseInt(line);
                    line = reader.readLine();
                }
        
            } catch (FileNotFoundException er) {
                throw new Error("Welp, couldn't read file -- FileNotFoundException.");
            } catch (IOException er) {
                throw new Error("Welp, couldn't read file -- IOException.");
            }
            arm = new Arm(
                    opmode.hardwareMap.dcMotor.get(SHOULDER_N),
                    home[0],
                    opmode.hardwareMap.dcMotor.get(ELBOW_N),
                    home[1],
                    (ModernRoboticsTouchSensor) opmode.hardwareMap.touchSensor.get(SHOULDERLIM_N),
                    (ModernRoboticsTouchSensor) opmode.hardwareMap.touchSensor.get(ELBOWLIM_N),
                    opmode);
        }
    }
    
    public void initAuto () {
        if (position == FieldPosition.ERROR) return;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
    
        imu = opmode.hardwareMap.get(BNO055IMU.class, IMU_N);
        imu.initialize(parameters);
    
        VisualsHandler.tempSetup(opmode);
        VisualsHandler.phoneLightOn();
        
        rollerMon = new Thread(new monitorRoller(), "roller monitor");
    }
    
    public void waitForStart () {
        ElapsedTime t = new ElapsedTime();
        
        t.reset();
        while (!imu.isGyroCalibrated() && !opmode.isStopRequested()) {
            if ((int)t.seconds() % 3 == 0) {
                if (position != FieldPosition.DEPOT_NO_SAMPLE) arrangement = VisualsHandler.takePreviewAndSample();
                arm.initloop(opmode.gamepad1.a);
                debug("Arrangement read as " + arrangement);
                tele("Status", "Calibrating gyro...");
                telewithup("Arrangement", arrangement);
            }
        }
        
        while (!opmode.isStarted() && !opmode.isStopRequested()) {
            if ((int)t.seconds() % 3 == 0) {
                if (position != FieldPosition.DEPOT_NO_SAMPLE) arrangement = VisualsHandler.takePreviewAndSample();
                arm.initloop(opmode.gamepad1.a);
                debug("Arrangement read as " + arrangement);
                tele("Status", "Initialized.");
                telewithup("Arrangement", arrangement);
            }
        }
    }
    
    /** TELEOP */
    public void start () {
        lock.open();
        markl.close();
        markr.close();
        roller.start();
    }
    
    /** CONTROLS
     * (#1) right stick x   robot strafe
     * right stick y        robot straight
     * left stick x         robot turn
     * left stick y         .
     * right button         .
     * left button          reverse robot direction
     * a                    out position
     * b                    silver position
     * x                    deposit mineral
     * y                    gold position
     * dup                  lift up
     * ddown                lift down
     * dleft                .
     * dright               .
     * right trigger        slow down
     * right bumper         speed up
     * left trigger         lift dd down
     * left bumper          lift dd up
     * back                 lock lander latcher
     * (#2) right stick x   .
     * right stick y        shoulder dd
     * left stick x         .
     * left stick y         elbow dd
     * right button         adjust shoulder offset
     * left button          adjust elbow offset
     * a                    deposit mineral
     * b                    silver position
     * x                    deposit mineral
     * y                    gold position
     * dup                  out position
     * ddown                close position
     * dleft                .
     * dright               .
     * right trigger        enable dd (hold with left trigger)
     * right bumper         roller dd intake
     * left trigger         enable dd (hold with right trigger)
     * left bumper          roller dd outtake
     * back                 reset arm encoders
     */
    public void drive () {
        if (opmode.gamepad1.right_bumper) currentMax = 1;
        else currentMax = MAX_DRIVE_POWER;
    
        double powerRF, powerRB, powerLF, powerLB;
        double
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
    
        lf.setPower(powerLF*currentMax);
        lb.setPower(powerLB*currentMax);
        rf.setPower(powerRF*currentMax);
        rb.setPower(powerRB*currentMax);
    
        arm.drive(
                opmode.gamepad1.a || opmode.gamepad2.dpad_up,
                opmode.gamepad2.dpad_down,
                opmode.gamepad1.b || opmode.gamepad2.b,
                opmode.gamepad1.y || opmode.gamepad2.y,
                opmode.gamepad2.right_stick_button,
                opmode.gamepad2.left_stick_button,
                opmode.gamepad2.right_trigger > .7 && opmode.gamepad2.left_trigger > .7,
                false,
                false,
                opmode.gamepad2.back,
                opmode.gamepad2.right_stick_y,
                opmode.gamepad2.left_stick_y
        );
    
        roller.drive(
                opmode.gamepad1.x || opmode.gamepad2.x || opmode.gamepad2.a,
                arm.getPos(),
                opmode.gamepad2.left_bumper,
                opmode.gamepad2.right_bumper,
                arm);
        
        latcher.run(opmode.gamepad1.dpad_up,
                    opmode.gamepad1.dpad_down,
                    opmode.gamepad1.left_bumper,
                    opmode.gamepad1.left_trigger > .7,
                    opmode.gamepad1.left_stick_button && opmode.gamepad1.right_stick_button);
        
        if (opmode.gamepad1.start && !locked) {
            lock.toggle();
            locked = true;
        } else if (locked && !opmode.gamepad1.start) locked = false;
    
        opmode.telemetry.addData("grabbed", roller.getGrabbed());
    
        arm.telemetry();
    
        opmode.telemetry.update();
    }
    
    public void stop () {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        roller.stop();
        arm.stop();
        latcher.stop();
    }
    public void autoStop () {
        stop();
        VisualsHandler.tempSetupStop();
    }
    
    /** AUTONOMOUS */
    
    public void auto () {
        if (!opmode.opModeIsActive()) return;
        
        rollerMon.start();
        
        lock.open();
        latcher.unlock();
        
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (t.seconds() < .25 && opmode.opModeIsActive()) debug("waiting...");
        
        latcher.raise();
        
        arm.auto_start();
        
        latcher.waitFor(opmode);
        
        double start = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        
        arm.up_quick();
        
        straightInches(-7, .5, .75);
        
        // 7 ccw to get middle (1.5 sec)
        // 32 cw to get right + backwards 3 inches (2 sec)
        // 46 ccs to get left + backwards 4 inches (2.5 sec)
        switch (arrangement) {
            case RIGHT:
                turnDegrees(32, .3, true, 1.5);
                break;
            case MIDDLE:
                turnDegrees(7, .2, false, .5);
                break;
            case LEFT:
                turnDegrees(46, .3, false, 2);
                break;
        }
    
        arm.close_pos();
    
        switch (arrangement) {
            case RIGHT:
            case LEFT:
                straightInches(-4, .4, .5);
                break;
        }
        
        latcher.lower();
        
        t.reset();
        while (t.seconds() < .75) debug("waiting...");
        
        straightInches(5, .3, 1);
        
        arm.rest();
        
        t.reset();
        while (t.seconds() < .25 && opmode.opModeIsActive()) debug("waiting...");
    
        switch (arrangement) {
            case RIGHT:
                straightInches(-8, .5, 1.5);
                turnDegrees(122, .3, false, 4);
                straightInches(-48, .6, 2.5);
                break;
            case MIDDLE:
                straightInches(-11, .5, 1.5);
                turnDegrees(83, .3, false, 4);
                straightInches(-48, .6, 2.5);
                break;
            case LEFT:
                straightInches(-7, .5, 1.5);
                turnDegrees(44, .3, false, 3);
                straightInches(-46, .6, 2.5);
                break;
        }
    
//        switch (arrangement) {
//            case RIGHT:
//                turnDegrees(42.5, .3, false, 2);
//                break;
//            default:
//                turnDegrees(45, .3, false, 2);
//                break;
//        }
            turnDegrees(135 + start - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
                        .3,
                        false,
                        1.5);
        
        
        // left 34 inches
        // middle 35 inches
        // right 32 inches
    
        if (position == FieldPosition.CRATER)
        switch (arrangement) {
            case RIGHT:
                straightInches(-32, .5, 1.5);
                break;
            case MIDDLE:
                straightInches(-35, .5, 1.5);
                break;
            case LEFT:
                straightInches(-34, .5, 1.5);
                break;
        }
        else if (position == FieldPosition.DEPOT)
            switch (arrangement) {
                case RIGHT:
                    straightInches(51, .5, 2);
                    break;
                case MIDDLE:
                    straightInches(54, .5, 2);
                    break;
                case LEFT:
                    straightInches(53, .5, 2);
                    break;
            }
        
        turnDegrees(90, .3, false, 3);
        
        markr.open();
        markl.open();
        
        t.reset();
        while (t.seconds() < .25 && opmode.opModeIsActive()) debug("waiting...");
        
        turnDegrees(90, .3, position == FieldPosition.DEPOT, 3);
        
        straightInches(-48, .6, 2);
        
        arm.out_pos();
        
        t.reset();
        while (t.seconds() < 1 && opmode.opModeIsActive()) debug("waiting...");
        
        latcher.waitFor(opmode);
        
        rollerMon.interrupt();
        
    }
    
    // todo

    private double currentPower = 0;
    private void straightInches (double inches, double maxPower, double seconds) {
        
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
        boolean _short = Math.abs(goalPosition) < SLOW_DOWN_START;
        
        t.reset();
        while (t.seconds() < seconds && opmode.opModeIsActive()) {
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
                if (currentPower < maxPower && slow >= SLOW_DOWN_START) {
                    currentPower += dt * PERCENT_PER_SECOND;
                    if (currentPower > maxPower) currentPower = maxPower;
                } else if (slow < SLOW_DOWN_START && currentPower > MIN_POWER) {
                    currentPower -= dt * PERCENT_PER_SECOND;
                    if (currentPower < MIN_POWER) currentPower = MIN_POWER;
                }
            } else {
                if (Math.abs((double) slow / goalPosition) > .75) {
                    currentPower += dt * PERCENT_PER_SECOND;
                    if (currentPower > maxPower) currentPower = maxPower;
                } else {
                    currentPower -= dt * PERCENT_PER_SECOND;
                    if (currentPower < MIN_POWER) currentPower = MIN_POWER;
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
    private void turnDegrees (double degrees, double maxPower, boolean clockwise, double seconds) {
        if (!opmode.opModeIsActive()) return;
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
        while (/*Math.abs(e) > thres &&*/ t.seconds() < seconds && opmode.opModeIsActive()) {
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
            
            
            double pi = e * dt, p = TURN_KP * e + TURN_KI * (sume + pi) + TURN_KD * ((e - laste)/dt);
            
            laste = e;
            
            //if (Math.abs(e) < Math.abs(toSlow)) accelPow = maxPower * (e/toSlow) + .1;

//            if (Math.abs(p) > accelPow && e * p >= 0) p = accelPow * dir;
//            else {
//                if (Math.abs(p) > accelPow) p = accelPow * dir;
//                sume += pi;
//            }
            
            //debug("dt: " + DECIMAL_FORMAT.format(dt) + "; error: " + DECIMAL_FORMAT.format(e) + "; p: " + DECIMAL_FORMAT.format(p)); //+ "; sum: " + DECIMAL_FORMAT.format(sume));
            debug("dt: " + dt + "; error: " + e + "; p: " + p + "; sum: " + DECIMAL_FORMAT.format(sume));
            
            //if (Math.abs(p) > maxPower) p = maxPower * dir;
            //p = Range.clip(p, -maxPower, maxPower);
            if (p > maxPower) {
                p = maxPower;
                if (e * p < 0) sume += pi;
            }
            else if (p < -maxPower) {
                p = -maxPower;
                if (e * p < 0) sume += pi;
            }
            else sume += pi;
            
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
    
    private double deltaTheta (double start, double current) {
        double ret = current - start;
        //debug(DECIMAL_FORMAT.format(start) + ", " + DECIMAL_FORMAT.format(current) + ", " + DECIMAL_FORMAT.format(ret));
        if (ret < -180) ret += 360;
        else if (ret > 180) ret -= 360;
        return ret;
    }
    
    /** DEBUGGING */
    private void tele (String caption, Object data) { opmode.telemetry.addData(caption, data); }
    private void teleup () { opmode.telemetry.update(); }
    private void telewithup (String caption, Object data) {opmode.telemetry.addData(caption, data); opmode.telemetry.update(); }
    
    private static void debug (String message) { Log.d(TAG, message); }
    private static void warn  (String message) { Log.w(TAG, message); }
    private static void error (String message) { Log.e(TAG, message); }
    
    class monitorRoller implements Runnable {
    
        @Override
        public void run () {
            while (!Thread.interrupted()) { roller.autodrive(); }
            roller.stop();
        }
        
    }
    
}
