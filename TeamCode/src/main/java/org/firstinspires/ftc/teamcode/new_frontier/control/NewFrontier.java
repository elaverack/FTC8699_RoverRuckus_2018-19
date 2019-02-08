package org.firstinspires.ftc.teamcode.new_frontier.control;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.FieldPosition;
import org.firstinspires.ftc.teamcode.Enums.SampleArrangement;
import org.firstinspires.ftc.teamcode.frontier.control.Latcher;
import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;

import java.text.DecimalFormat;

public class NewFrontier {
    
    /** CONSTANTS */
    private static final String
            TAG             = "New Frontier",
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
            UNLOCK_P                = .85, // TODO
            LOCK_P                  = .95, // TODO
            MARK_LEFT_HOLD          = .25, // TODO
            MARK_LEFT_DROP          = .5,  // TODO
            MARK_RIGHT_HOLD         = .16, // TODO
            MARK_RIGHT_DROP         = .4,  // TODO
            //TURN_KP                 = 6,
            //TURN_KI                 = 2.5,
            //TURN_KD                 = 0.02,
            STRAFE_INCHES_PER_REV   = 8.75;
    
    private static final int
            ENCODER_PER_REV = 560;
    
    private static DecimalFormat DECIMAL_FORMAT = new DecimalFormat("#.##");
    
    /** INSTANCE VARS */
    private LinearOpMode opmode;
    private DcMotor rf, rb, lf, lb;
    private ToggleServo markl, markr, lock;
    private Latcher latcher;
    private Arm arm;
    private SingleRoller roller;
    
    private double powerRF, powerRB, powerLF, powerLB;
    
    private BNO055IMU         imu;
    private FieldPosition     position    = FieldPosition.ERROR;
    private SampleArrangement arrangement = SampleArrangement.ERROR;
    
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
    
        arm = new Arm(
                opmode.hardwareMap.dcMotor.get(SHOULDER_N),
                opmode.hardwareMap.dcMotor.get(ELBOW_N),
                (ModernRoboticsTouchSensor) opmode.hardwareMap.touchSensor.get(SHOULDERLIM_N),
                (ModernRoboticsTouchSensor) opmode.hardwareMap.touchSensor.get(ELBOWLIM_N),
                opmode);
    
        roller = new SingleRoller(
                opmode.hardwareMap.crservo.get(ROLLER_N),
                (ModernRoboticsI2cColorSensor) opmode.hardwareMap.colorSensor.get(COLOR_N));
        
        arm.init();
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
    }
    
    public void waitForStart () {
        ElapsedTime t = new ElapsedTime();
        
        t.reset();
        while (!imu.isGyroCalibrated() && opmode.opModeIsActive()) {
            if ((int)t.seconds() % 3 == 0) {
                if (position != FieldPosition.DEPOT_NO_SAMPLE) arrangement = VisualsHandler.takePreviewAndSample();
                debug("Arrangement read as " + arrangement);
                tele("Status", "Calibrating gyro...");
                telewithup("Arrangement", arrangement);
            }
        }
        
        while (!opmode.isStarted() && opmode.opModeIsActive()) {
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
        markl.close();
        markr.close();
        roller.start();
    }
    
    /** Controls:
     * (#1) right stick x   robot strafe
     * right stick y        robot straight
     * left stick x         robot turn
     * left stick y         .
     * right button         .
     * left button          .
     * a                    .
     * b                    .
     * x                    .
     * y                    .
     * dup                  lift up
     * ddown                lift down
     * dleft                .
     * dright               .
     * right trigger        slow down
     * right bumper         speed up
     * left trigger         lift dd down
     * left bumper          lift dd up
     * (#2) right stick x   .
     * right stick y        .
     * left stick x         .
     * left stick y         .
     * right button         .
     * left button          .
     * a                    .
     * b                    .
     * x                    .
     * y                    .
     * dup                  .
     * ddown                .
     * dleft                .
     * dright               .
     * right trigger        .
     * right bumper         .
     * left trigger         .
     * left bumper          .
     */
    public void drive () {
    
    }
    
    
    /** DEBUGGING */
    /** DEBUGGING */
    private void tele (String caption, Object data) { opmode.telemetry.addData(caption, data); }
    private void teleup () { opmode.telemetry.update(); }
    private void telewithup (String caption, Object data) {opmode.telemetry.addData(caption, data); opmode.telemetry.update(); }
    
    private static void debug (String message) { Log.d(TAG, message); }
    private static void warn  (String message) { Log.w(TAG, message); }
    private static void error (String message) { Log.e(TAG, message); }
    
}
