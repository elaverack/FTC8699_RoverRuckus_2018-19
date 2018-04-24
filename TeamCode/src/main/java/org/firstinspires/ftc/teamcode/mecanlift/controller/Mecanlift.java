package org.firstinspires.ftc.teamcode.mecanlift.controller;

// Created on 1/28/2018 at 11:29 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift

import android.util.Log;
import android.view.View;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtUltrasonicSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.visuals.AlignmentCircle;
import org.firstinspires.ftc.teamcode.visuals.Vector3;
import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;
import org.firstinspires.ftc.teamcode.visuals.VuforiaHandler;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Enums.*;
import static org.firstinspires.ftc.teamcode.visuals.Vector3.round;

public class Mecanlift {

    /** CONSTANTS */
    private static final double                         // Grabber servo positions
            blo = 0.235,//157,                          // Bottom left open
            blc = 0.421,                                // Bottom left close
            bro = 0.569,                                // Bottom right open
            brc = 0.4406,                               // Bottom right close
            tlo = 0.804,                                // Top left open
            tlc = 0.627,                                // Top left close
            tro = 0.255,                                // Top right open
            trc = 0.412,                                // Top right close

            jewelArm_down = 0,                          // The jewel servo position for lowering the arm
            jewelArm_up = .793,                         // The jewel servo position for raising the arm

            relic_grabber_up = 0.863,                   // The relic up/down servo position for raising the grabber
            relic_grabber_down = 0.255,                 // The relic up/down servo position for lowering the grabber
            relic_grabber_grab = 0.392,                 // The relic grabber servo position for grabbing the relic
            relic_grabber_release = 0.588,              // The relic grabber servo position for releasing the relic

            strafing_inches_per_rev = 10.34,            // Inches strafed for one wheel revolution
            drive_distance_acceleration = 1.0/6000.0,   // Acceleration for driving distances

            div_threshold = 100;                         // Threshold for combining divs in picture

    private static final int
            flip_position = 1000,                       // Position of lift when flipping from ground position

            ccwJewelAngle = 10,                         // Gyro angle to turn to when turning counter-clockwise to hit off jewel
            cwJewelAngle = 350,                         // Gyro angle to turn to when turning clockwise to hit off jewel

            angle_thres = 5,                            // Angle threshold for turning using gyro

            indicatorPort = 0;

    private static final AlignmentCircle
            right = new AlignmentCircle(new Point(641, 1105), 80),
            left = new AlignmentCircle(new Point(378, 1107), 80);

    private static final I2cController.I2cPortReadyCallback callback = new I2cController.I2cPortReadyCallback() {
        @Override
        public void portIsReady(int port) { debug("I'm amazing..."); }
    };

    private final Runnable bottomROC = new Runnable() {
        @Override
        public void run() {
            if (!rot.flipped) lift.grabbedBlock();
        }
    }, topROC = new Runnable() {
        @Override
        public void run() {
            if (rot.flipped) lift.grabbedBlock();
        }
    }, runOnOpen = new Runnable() {
        @Override
        public void run() {
            lift.releasedBlocks();
        }
    };

    private static final String
            TAG = "Mecanlift",                          // Debugging tag

            drive_rfN = "rf",                           // The front right motor name
            drive_rbN = "rb",                           // The back right motor name
            drive_lfN = "lf",                           // The front left motor name
            drive_lbN = "lb",                           // The back left motor name

            // Note: Looking at robot, not from
            grabber_blN = "bl",                         // The bottom left grabber servo name
            grabber_brN = "br",                         // The bottom right grabber servo name
            grabber_tlN = "tl",                         // The top left grabber servo name
            grabber_trN = "tr",                         // The top right grabber servo name
            grabber_rotN = "rot",                       // The rotational motor name

            liftN = "lift",                             // The lift motor name

            relicArmMotorN = "relic",                   // The name of the motor to move relic arm
            liftRelicServoN = "up",                     // The name of the servo to lift the relic up
            grabRelicServoN = "grab",                   // The name of the servo to grab the relic

            jewelN = "jewel",                           // The jewel arm servo name
            colorN = "color",                           // The jewel arm color sensor name
            ultrasonicSensorN = "ultra",                // The ultrasonic sensor name
            gyroN = "gyro",                             // The gyro sensor name

            dimN = "dim",                               // The device interface module name
            topBeamN = "tbeam",                         // The top beam sensor name
            bottomBeamN = "bbeam";                      // The bottom beam sensor name

    /** VARIABLES */
    /** OPMODE */
    private OpMode opmode;

    /** DRIVE */
    private DcMotor rf, rb, lf, lb;
    private float powerRF, powerRB, powerLF, powerLB;
    // Variables for quick turn
    private boolean turned = false, turning = false, past = false;
    private int qt_angle = 0;

    /** GRABBER */
    private ToggleServo tl, tr, bl, br;
    private Rotater rot;

    /** LIFT */
    private Lift lift;

    /** VISUALS */
    private VisualsHandler visuals;
    private ElapsedTime time;

    /** SENSORS */
    private GyroSensor gyro; //NOTE: counter clockwise is positive for gyro
    private ModernRoboticsI2cColorSensor jewelSensor;
    private HiTechnicNxtUltrasonicSensor ultra;
    private DeviceInterfaceModule dim;
    private TouchSensor topBeam, bottomBeam;
    private boolean indOn = false;

    /** JEWELS */
    private Servo jewelArm;

    /** FOR AUTONOMOUS USE */
    private Color allianceColor;
    private Position alliancePosition;
    private Column keyColumn = Column.ERROR;
    private boolean vumarkPlaced = false, doneAligning = false, intro_shown = false;
    public VuforiaHandler.PosRot lastKnownPos;

    /** INITIALIZERS */
    public Mecanlift (OpMode om) { init(om, false, Color.ERROR, Position.ERROR); } // USE IF TELEOP
    public Mecanlift (OpMode om, Color allianceColor, Position alliancePosition) { init(om, true, allianceColor, alliancePosition); }
    private void init (OpMode om, boolean auto, Color ac, Position ap) {
        debug("---- BEGIN INIT ----");

        opmode = om;
        allianceColor = ac;
        alliancePosition = ap;

        /** DRIVE */
        debug("Setting up drive...");

        rf = opmode.hardwareMap.dcMotor.get(drive_rfN);
        rb = opmode.hardwareMap.dcMotor.get(drive_rbN);
        lf = opmode.hardwareMap.dcMotor.get(drive_lfN);
        lb = opmode.hardwareMap.dcMotor.get(drive_lbN);

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        debug("Done.");

        /** LIFT */
        debug("Setting up lift...");

        lift = new Lift(opmode.hardwareMap.dcMotor.get(liftN));

        debug("Done.");

        /** GRABBER */
        debug("Setting up grabber...");

        bl = new ToggleServo(opmode.hardwareMap.servo.get(grabber_blN), blo, blc, bottomROC, runOnOpen);
        br = new ToggleServo(opmode.hardwareMap.servo.get(grabber_brN), bro, brc, bottomROC, runOnOpen);
        tl = new ToggleServo(opmode.hardwareMap.servo.get(grabber_tlN), tlo, tlc, topROC, runOnOpen);
        tr = new ToggleServo(opmode.hardwareMap.servo.get(grabber_trN), tro, trc, topROC, runOnOpen);
        rot = new Rotater(opmode.hardwareMap.dcMotor.get(grabber_rotN), lift);

        debug("Done.");

        /** RELIC ARM */
        debug("Setting up relic arm...");

        initRelicArm();

        debug("Done.");

        /** JEWEL ARM */
        debug("Setting up jewel arm...");

        jewelArm = opmode.hardwareMap.servo.get(jewelN);

        debug("Done.");

        /** AUTONOMOUS */
        debug("Setting up teleop sensors...");

        gyro = opmode.hardwareMap.gyroSensor.get(gyroN);
        dim = opmode.hardwareMap.deviceInterfaceModule.get(dimN);
        dim.setDigitalChannelMode(indicatorPort, DigitalChannel.Mode.OUTPUT);
        dim.setDigitalChannelState(indicatorPort, false);

        topBeam = opmode.hardwareMap.touchSensor.get(topBeamN);
        bottomBeam = opmode.hardwareMap.touchSensor.get(bottomBeamN);

        debug("Done.");

        if (auto) {
            /** VISUALS */
            debug("Setting up visuals...");

            visuals = new VisualsHandler(opmode, false);
            VisualsHandler.phoneLightOn();

            visuals.right = right;
            visuals.left = left;
            visuals.showAlignmentCircles();
            activateVuforia();
            time = new ElapsedTime();

            visuals.getPreview().setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) { doneAligning = true; visuals.getPreview().setOnClickListener(null); }
            });

            debug("Done.");

            /** SENSORS */

            debug("Setting up color sensor...");

            jewelSensor = (ModernRoboticsI2cColorSensor)opmode.hardwareMap.colorSensor.get(colorN);
            jewelSensor.resetDeviceConfigurationForOpMode();
            csLightOn();

            debug("Done.");

            debug("Setting up ultrasonic sensor...");

            ultra = (HiTechnicNxtUltrasonicSensor) opmode.hardwareMap.ultrasonicSensor.get(ultrasonicSensorN);
            ultra.resetDeviceConfigurationForOpMode();

            debug("Done.");
        } else csLightOff();
    }

    public void calibrateGyro () {
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            opmode.telemetry.addData("Status", "Waiting on gyro...");
            opmode.telemetry.update();
        }
    }
    public int theta() { return gyro.getHeading(); }
    private int specialTheta() {
        int theta = theta();
        if (theta > 180) theta -= 360;
        return theta;
    }
    private double getUltraInches () { return ultra.getDistance(DistanceUnit.INCH); }

    private void csLightOff() { opmode.hardwareMap.colorSensor.get(colorN).enableLed(false); }
    private void csLightOn() { opmode.hardwareMap.colorSensor.get(colorN).enableLed(true); }

    private void indicatorOn () {dim.setDigitalChannelState(indicatorPort, true); indOn = true;}
    private void indicatorOff () {dim.setDigitalChannelState(indicatorPort, false); indOn = false;}

    private void activateVuforia () { visuals.vuforia.start(); }
    private String keyColumn () { return keyColumn.toString(); }
    private String checkColumn () { keyColumn = Column.look(visuals.vuforia); return keyColumn(); }

    /** TELEOP */
    /** INIT LOOP METHODS */
    public void showAligning () { if (((int)(time.seconds()%2) == 1)) { visuals.showAlignmentCircles(); intro_shown = false; } }
    public void doFullInitLoop () {
        tele("Seeing?", checkColumn());
        if (doneAligning) {
            if (!intro_shown) { visuals.showIntro(); intro_shown = true; }
            tele("Vumark Placed?", vumarkPlaced);
            VuforiaHandler.PosRot pr = visuals.vuforia.getRelativePosition();
            if (pr.position.y != 0 && lastKnownPos == null) {
                lastKnownPos = pr;
                vumarkPlaced = true;
            } else if (pr.position.y != 0) {
                lastKnownPos.doAverage(pr);
                lastKnownPos.position.teleout(opmode, "Pos");
                lastKnownPos.rotation.teleout(opmode, "Rot");
            } else if (vumarkPlaced) {
                lastKnownPos = null;
                vumarkPlaced = false;
            }
            if (((int)(time.seconds()%4) == 0)) visuals.checkJewelsWithCamera();
            tele("Phone jewels", visuals.jewelConfig.toString());
            tele("Ultra", getUltraInches());
        } else showAligning();
        teleup();
    }

    /** START METHODS */
    public void start () { raiseArm(); bl.open(); br.open(); tl.open(); tr.open(); startRelicArm(); lift.start(); }

    /** LOOP METHODS */
    public void drive() { // New controls of 2/12

        operateMechanisms();

        /** MECANUM WHEELS */
        runDrive();

    }
    public void drive(boolean debug_drive) {
        drive();
        if (debug_drive) {
            opmode.telemetry.addData("Joys", String.format("y: %1$s, x: %2$s, r: %3$s",
                    round(-opmode.gamepad1.right_stick_y),
                    round(opmode.gamepad1.right_stick_x),
                    round(opmode.gamepad1.left_stick_x)
            ));
            opmode.telemetry.addData("Pows", String.format("rf: %1$s, rb: %2$s, lf: %3$s, lb: %4$s",
                    round(powerRF), round(powerRB), round(powerLF), round(powerLB)
            ));
        }
    }
    public void runDrive () {
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
        doQuickTurn(opmode.gamepad1.left_stick_button);
        if (powerRF > 1) {powerRF = 1f;} else if (powerRF < -1) {powerRF = -1f;}
        if (powerRB > 1) {powerRB = 1f;} else if (powerRB < -1) {powerRB = -1f;}
        if (powerLF > 1) {powerLF = 1f;} else if (powerLF < -1) {powerLF = -1f;}
        if (powerLB > 1) {powerLB = 1f;} else if (powerLB < -1) {powerLB = -1f;}

        if (opmode.gamepad1.right_trigger > 0.5) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }

        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);
    }
    public void operateMechanisms() {
        /** LIFT */
        lift.run(lift_pos_tog(), lift_ground(), lift_direct_drive_up(), lift_direct_drive_down(),
                (opmode.gamepad2.right_stick_button && opmode.gamepad2.left_stick_button));

        /** GRABBER */
        boolean a = toggle_bottom(), b = toggle_top();
        if (!rot.flipped) {
            bl.tob(a);
            br.tob(a);
            tl.tob(b);
            tr.tob(b);
        } else {
            bl.tob(b);
            br.tob(b);
            tl.tob(a);
            tr.tob(a);
        }
        rot.doRotation(flip_grabber());
        rot.doStraighten(rot_straighten());
        rot.doRotFix(fix_rotate());
        rot.stopRotation(opmode.gamepad2.dpad_right && (opmode.gamepad2.right_trigger < .5));

        doRelicArm(
                opmode.gamepad2.right_stick_y,
                opmode.gamepad2.right_trigger,
                opmode.gamepad2.y,
                opmode.gamepad2.x
        );
    }
    public void runIndicator () {
        if (!rot.flipped) {
            if (indOn && bottomBeam.isPressed()) indicatorOff();
            else if (!indOn && !bottomBeam.isPressed()) indicatorOn();
        } else {
            if (indOn && topBeam.isPressed()) indicatorOff();
            else if (!indOn && !topBeam.isPressed()) indicatorOn();
        }
    }
    public void runAutoMechanisms () {
        /** LIFT */
        lift.run(lift_pos_tog(), lift_ground(), lift_direct_drive_up(), lift_direct_drive_down(),
                (opmode.gamepad2.right_stick_button && opmode.gamepad2.left_stick_button));

        /** GRABBER */
        boolean a = opmode.gamepad1.a || opmode.gamepad2.a, b = toggle_top();
        if (!rot.flipped) {
            bl.tob(a);
            br.tob(a);
            tl.tob(b);
            tr.tob(b);
        } else {
            bl.tob(b);
            br.tob(b);
            tl.tob(a);
            tr.tob(a);
        }
        runIndicator();
        if (!(opmode.gamepad1.left_trigger > .5 || opmode.gamepad2.back)) {
            if (!rot.flipped && br.isOpened() && !bottomBeam.isPressed()) {
                br.close();
                bl.close();
            } else if (rot.flipped && tr.isOpened() && !topBeam.isPressed()) {
                tr.close();
                tl.close();
            }
        } else if (!br.isOpened() || !tr.isOpened()) {
            br.open();
            bl.open();
            tr.open();
            tl.open();
        }

        rot.doRotation(flip_grabber());
        rot.doStraighten(rot_straighten());
        rot.doRotFix(fix_rotate());
        rot.stopRotation(opmode.gamepad2.dpad_right && (opmode.gamepad2.right_trigger < .5));

        doRelicArm(
                opmode.gamepad2.right_stick_y,
                opmode.gamepad2.right_trigger,
                opmode.gamepad2.y,
                opmode.gamepad2.x
        );

    }

    /** STOP METHODS */
    public void stop() {

        /** DRIVE */
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);

        /** RELIC ARM */
        stopRelicArm();

        /** LIFT */
        lift.stop();

        /** GRABBER */
        rot.stop();

    }
    public void closeVisuals () { visuals.close(); }

    /** JEWEL METHODS */
    private void raiseArm() { jewelArm.setPosition(jewelArm_up); }
    private void lowerArm() { jewelArm.setPosition(jewelArm_down); }

    /** QUICK TURN */
    private void doQuickTurn (boolean b) {
        if (turning) {
            if (!past && gyro.getHeading() < qt_angle) { past = true; }
            if (past && gyro.getHeading() > qt_angle) {
                turning = false;
                past = false;
                return;
            }
            powerRF += 1f;
            powerRB += 1f;
            powerLF -= 1f;
            powerLB -= 1f;
            return;
        }
        if (turned && !b) { turned = false; return; }
        if (!b) return;
        if (!turned) {
            turning = true;
            qt_angle = gyro.getHeading() + 165;
            if (qt_angle > 359) qt_angle -= 360;
            powerRF += 1f;
            powerRB += 1f;
            powerLF -= 1f;
            powerLB -= 1f;
        }
    }

    /** RELIC ARM */
//    grab: ng 150, g 100
//    up: d 65, u 220
//    out/in: tetrix motor
            // This is super lazy programming. Sorry...
    private DcMotor relicMotor;
    private ToggleServo grab, up;
    private void initRelicArm() {
        relicMotor = opmode.hardwareMap.dcMotor.get(relicArmMotorN);
        relicMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        relicMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grab = new ToggleServo(opmode.hardwareMap.servo.get(grabRelicServoN), relic_grabber_release, relic_grabber_grab);
        up = new ToggleServo(opmode.hardwareMap.servo.get(liftRelicServoN), relic_grabber_down, relic_grabber_up);
    }
    private void startRelicArm() {
        relicMotor.setPower(0);
        up.open();
        grab.open();
    }
    private void doRelicArm (float motorPower, float relicTrigger, boolean togGrab, boolean togUp) {
        relicMotor.setPower(motorPower * relicTrigger);
        grab.tob(togGrab);
        up.tob(togUp);
    }
    private void stopRelicArm () { relicMotor.setPower(0); }

    /** GAMEPADS */
//    gamepad 1:
//    UP			        : lift pos. toggle
//    DOWN			        : lift ground
//    RIGHT+RB		        : fix rotater
//    LT			        : toggle bottom
//    LB			        : toggle top
//    LJOY			        : rotation
//    LJOYBUTTON		    : turn around
//    RJOY			        : movement
//    RJOYBUTTON		    : flip grabber
//    RT			        : slow down
//    Y			            : lift direct drive up
//    X			            : lift direct drive down
//    B			            : toggle top
//    A			            : toggle bottom
//
//    gamepad 2:
//    UP			        : lift pos. toggle
//    DOWN			        : lift ground
//    RIGHT+RT		        : fix rotater
//    LT			        : lift direct drive down
//    LB			        : lift direct drive up
//    LJOYBUTTON+RJOYBUTTON	: set lift ground position
//    RB			        : flip grabber
//    Y			            : na
//    X			            : na
//    B			            : toggle top
//    A			            : toggle bottom
    private boolean[]
            lpt     = new boolean[]{false, false}, // lift pos tog
            lg      = new boolean[]{false, false}, // lift ground
            fr      = new boolean[]{false, false}, // fix rotate
            rs      = new boolean[]{false, false},
            tb      = new boolean[]{false, false}, // toggle bottom
            tt      = new boolean[]{false, false}, // toggle top
            fg      = new boolean[]{false, false}; // flip grabber
    private boolean lift_pos_tog () {
        if (opmode.gamepad1.dpad_up && !lpt[0]) {
            lpt[0] = true;
            return true;
        } else if (!opmode.gamepad1.dpad_up && lpt[0]) lpt[0] = false;
        if (opmode.gamepad2.dpad_up && !lpt[1]) {
            lpt[1] = true;
            return true;
        } else if (!opmode.gamepad2.dpad_up && lpt[1]) lpt[1] = false;
        return false;
    }
    private boolean lift_ground () {
        if (opmode.gamepad1.dpad_down && !lg[0]) {
            lg[0] = true;
            return true;
        } else if (!opmode.gamepad1.dpad_down && lg[0]) lg[0] = false;
        if (opmode.gamepad2.dpad_down && !lg[1]) {
            lg[1] = true;
            return true;
        } else if (!opmode.gamepad2.dpad_down && lg[1]) lg[1] = false;
        return false;
    }
    private boolean fix_rotate () {
        if ((opmode.gamepad1.dpad_right && opmode.gamepad1.right_bumper) && !fr[0]) {
            fr[0] = true;
            return true;
        } else if (!(opmode.gamepad1.dpad_right && opmode.gamepad1.right_bumper) && fr[0]) fr[0] = false;
        if ((opmode.gamepad2.dpad_right && opmode.gamepad2.right_trigger > .5) && !fr[1]) {
            fr[1] = true;
            return true;
        } else if (!(opmode.gamepad2.dpad_right && opmode.gamepad2.right_trigger > .5) && fr[1]) fr[1] = false;
        return false;
    }
    private boolean rot_straighten () {
        if (opmode.gamepad1.dpad_left && !rs[0]) {
            rs[0] = true;
            return true;
        } else if (!opmode.gamepad1.dpad_left && rs[0]) rs[0] = false;
        if (opmode.gamepad2.dpad_left && !rs[1]) {
            rs[1] = true;
            return true;
        } else if (!opmode.gamepad2.dpad_left && rs[1]) rs[1] = false;
        return false;
    }
    private boolean toggle_bottom () {
        if ((opmode.gamepad1.a || opmode.gamepad1.left_trigger > .4) && !tb[0]) {
            tb[0] = true;
            return true;
        } else if (!(opmode.gamepad1.a || opmode.gamepad1.left_trigger > .4) && tb[0]) tb[0] = false;
        if (opmode.gamepad2.a && !tb[1]) {
            tb[1] = true;
            return true;
        } else if (!opmode.gamepad2.a && tb[1]) tb[1] = false;
        return false;
    }
    private boolean toggle_top () {
        if ((opmode.gamepad1.b || opmode.gamepad1.left_bumper) && !tt[0]) {
            tt[0] = true;
            return true;
        } else if (!(opmode.gamepad1.b || opmode.gamepad1.left_bumper) && tt[0]) tt[0] = false;
        if (opmode.gamepad2.b && !tt[1]) {
            tt[1] = true;
            return true;
        } else if (!opmode.gamepad2.b && tt[1]) tt[1] = false;
        return false;
    }
    private boolean flip_grabber () {
        if (opmode.gamepad1.right_stick_button && !fg[0]) {
            fg[0] = true;
            return true;
        } else if (!opmode.gamepad1.right_stick_button && fg[0]) fg[0] = false;
        if (opmode.gamepad2.right_bumper && !fg[1]) {
            fg[1] = true;
            return true;
        } else if (!opmode.gamepad2.right_bumper && fg[1]) fg[1] = false;
        return false;
    }
    private boolean lift_direct_drive_up () { return opmode.gamepad1.y || opmode.gamepad2.left_bumper; }
    private boolean lift_direct_drive_down () { return opmode.gamepad1.x || opmode.gamepad2.left_trigger > .5; }

    /** AUTONOMOUS */
    public void doFullAutonomous (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) { closeVisuals(); return; }

        debug("---- BEGIN AUTONOMOUS ----");

        ElapsedTime localTime = new ElapsedTime();

        if (lastKnownPos == null) {
            lastKnownPos = visuals.vuforia.getRelativePosition();

            warn("Pos was null. Replaced with zeros.");
        }
        lastKnownPos.toInches();

        telewithup("Status", "Doing jewels...");
        debug("Doing jewels.");

        doJewels(opmode);

        if (opmode.opModeIsActive() && !alliancePosition.justDoJewel() && !allianceColor.parkAuto()) {
            debug("NOT JUST DO JEWELS AND NOT PARK!");

            telewithup("Status", "Driving off balancing plate...");
            debug("Driving off balancing stone...");

            driveOffBalance(opmode);

            telewithup("Status", "Driving to cryptobox...");
            debug("Driving to cryptobox...");

            driveToCryptobox(opmode);

            telewithup("Status", "Driving to column...");
            debug("Driving to column...");
            driveToColumn(opmode);

            telewithup("Status", "Placing glyph...");
            debug("Placing glyph...");
            placeGlyph(opmode);
        }

        if (opmode.opModeIsActive() && !alliancePosition.justDoJewel() && allianceColor.parkAuto()) {
            debug("PARK ONLY AUTONOMOUS!");

            VisualsHandler.phoneLightOff();
            doPark(opmode);
        }

        VisualsHandler.phoneLightOff();

        lift.groundground();
        lift.waitForEncoders(opmode);

        debug("Done with autonomous. Time was " + localTime.seconds() + " seconds.");

        while (opmode.opModeIsActive()) {
            tele("Status", "Done.");
            lastKnownPos.position.teleout(opmode, "Pos");
            lastKnownPos.rotation.teleout(opmode, "Rot");
            tele("Θ", specialTheta());
            tele("Ultra", getUltraInches());
            telewithup("Column", keyColumn());
        }

        closeVisuals();
    }

    private void doJewels (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;
        boolean turnCCW;

        debug("-- BEGIN DOING JEWELS --");

        ElapsedTime localTime = new ElapsedTime();

        JewelConfig pre = visuals.jewelConfig;
        debug("Jewel config from initialization: " + pre.toString());

        telewithup("Status", "Closing/lowering everything...");
        debug("Setting everything up...");

        raiseArm();
        br.close();
        bl.close();
        tr.close();
        tl.close();
        lift.setPosition(flip_position+250);

        debug("Checking jewels with camera...");
        time.reset();

        visuals.checkJewelsWithCamera();
        JewelConfig post = visuals.jewelConfig;

        debug("Done checking. Time was " + time.seconds() + " seconds.");
        debug("Jewel config after initialization: " + post.toString());
        debug("Lowering arm...");

        lowerArm();

        debug("Calibrating gyro...");

        calibrateGyro();

        telewithup("Status", "Reading color sensor...");
        debug("Reading color sensor...");

        Color jewelC = Color.readColorSensor(jewelSensor);

        debug("Jewel color of sensor: " + jewelC.toString());

        if (jewelC == Color.ERROR) {
            telewithup("Status", "Couldn't read it with color sensor...");
            warn("Couldn't read jewels with color sensor.");
            if (post == JewelConfig.ERROR) {
                telewithup("Status", "Couldn't read it actively with camera...");
                warn("Couldn't get jewel color while running...");
                if (pre == JewelConfig.ERROR) {
                    telewithup("Status", "Couldn't read it...");
                    error("Couldn't get jewel color at all.");
                    raiseArm();
                    debug("Done doing jewels. Time is " + localTime.seconds() + " seconds.");
                    return;
                } else turnCCW = allianceColor.turnCCWforJewels(pre);
            } else turnCCW = allianceColor.turnCCWforJewels(post);
        } else turnCCW = allianceColor.turnCCWforJewels(jewelC);

        debug("Waiting for lift...");

        lift.waitForEncoders(opmode);

        if (turnCCW) {
            telewithup("Status", "Turning left...");
            debug("Knocking off left jewel...");

            turnCentrallyPastAngle(opmode, ccwJewelAngle, .3f);
        } else {
            telewithup("Status", "Turning right...");
            debug("Knocking off right jewel...");

            turnCentrallyPastAngle(opmode, cwJewelAngle, .3f);
        }

        telewithup("Status", "Raising arm...");
        debug("Raising arm...");

        raiseArm();

        telewithup("Status", "Completed doing jewel.");
        debug("Done doing jewels. Time is " + localTime.seconds() + " seconds.");
    }

    private void driveOffBalance (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        debug("-- DRIVING OFF BALANCING STONE --");

        ElapsedTime localTime = new ElapsedTime();

        debug("Fixing rotation...");

        turnCentrallyToAngle(opmode, (int)lastKnownPos.rotation.z, .2f, .15f);

        if (alliancePosition == Position.CORNER) {
            debug("Strafing 27 inches...");

            strafeDistanceAbsDrift(opmode, 27, allianceColor == Color.RED);

            if (allianceColor == Color.BLUE) {
                debug("Straightening up to drive 8 inches...");

                turnCentrallyToAngle(opmode, (int)lastKnownPos.rotation.z, .2f, .15f);
            }
        } else {
            double d = 0;
            if (allianceColor == Color.BLUE) d = -3;

            debug("Strafing " + (34 + d) + " inches...");

            strafeDistanceAbsDrift(opmode, 34 + d, allianceColor == Color.RED);

            debug("Straightening up...");

            turnCentrallyToAngle(opmode, (int)lastKnownPos.rotation.z, .2f, .15f);
        }

        debug("Done driving off balancing stone. Time is " + localTime.seconds() + " seconds.");


    }

    private void driveToCryptobox (LinearOpMode opmode) {
        if (!opmode.opModeIsActive() || lastKnownPos == null) return;

        debug("-- DRIVING TO CRYPTOBOX --");

        ElapsedTime localTime = new ElapsedTime();

        if (alliancePosition == Position.CORNER) {
            if (allianceColor == Color.BLUE) { debug("Driving towards cryptobox..."); driveDistance(opmode, 10, .5f); }

            debug("Turning away from cryptobox...");

            turnCentrallyToAngle(opmode, alliancePosition.getAwayFromCryptoAngle(allianceColor, lastKnownPos.rotation.z), .4f, .2f);
        }

        debug("Done driving to cryptobox. Time is " + localTime.seconds() + " seconds.");

    }

    private void driveToColumn (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        debug("-- DRIVING TO KEY COLUMN --");

        ElapsedTime localTime = new ElapsedTime();

        debug("Waiting a second before processing cryptobox...");

        wait(opmode, 1);

        double ultra = getUltraInches();

        debug("Saved ultra at " + ultra + "inches.");
        debug("Processing cryptobox...");

        time.reset();
        double[] coordinates = processDividerCoordinates(visuals.previewCryptobox(allianceColor));
        telewithup("NO divs", coordinates.length);

        debug("Time to process cryptobox is " + time.seconds());
        for (double x : coordinates) debug("Got divider: " + x);

        double d = keyColumn.doCryptoboxLogic(allianceColor, alliancePosition, coordinates);

        debug("Distance to drive is " + d + " inches.");

        strafeDistance(opmode, d, allianceColor == Color.RED && alliancePosition == Position.CORNER);

        if (getUltraInches() < ultra) ultra = getUltraInches();
        if (ultra < 12.5) { debug("Driving away from cryptobox..."); driveDistance(opmode, 12.5 - ultra, .3f); }

        debug("Turning back around...");

        turnCentrallyToAngle(opmode, alliancePosition.getFacingCryptoAngle(allianceColor, lastKnownPos.rotation.z), .5f, .2f);

        debug("Done driving to key column. Time is " + localTime.seconds() + " seconds.");

    }

    private void placeGlyph (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        debug("-- BEGIN PLACING GLYPH --");

        ElapsedTime localTime = new ElapsedTime();

        debug("Grounding lift...");

        lift.ground();
        lift.waitForEncoders(opmode);

        debug("Driving a foot...");

        driveDistance(opmode, 12, .3f);

        debug("Opening everything...");

        bl.open();
        br.open();
        tl.open();
        tr.open();

        debug("Backing up...");

        driveDistance(opmode, -4, .3f);

        debug("Actually grounding lift...");

        lift.groundground();
        lift.waitForEncoders(opmode);

        debug("Done. Time was " + localTime.seconds() + " seconds.");
    }

    private void doPark (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;
        lift.setPosition(flip_position);
        turnCentrallyPastAngle(opmode, alliancePosition.getParkingAngle(allianceColor), .5f);
        lift.waitForEncoders(opmode);
        driveDistance(opmode, alliancePosition.getParkingDistance(), .5f);
        lift.groundground();
        lift.waitForEncoders(opmode);
    }

    /** AUTONOMOUS HELPERS */
    /** DRIVE DISTANCE */
    private void driveDistance (LinearOpMode opmode, double inches, float power) {
        if (!opmode.opModeIsActive()) return;
        int encoder = (int)(1120 * (inches/(Math.PI * 4.0)));
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setTargetPosition(encoder);
        rb.setTargetPosition(encoder);
        lf.setTargetPosition(encoder);
        lb.setTargetPosition(encoder);
        setDrivePower(power);
        while (!Lift.update_encoders(rb) && opmode.opModeIsActive())
            telewithup("Status", "Driving straight " + Vector3.round(((float)inches)) + " inches...");
        rf.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public Vector3 driveDistanceDrift (LinearOpMode opmode, double inches, float power) {
        if (!opmode.opModeIsActive()) return new Vector3();
        int e_goal = (int)(1120.0 * (inches/(Math.PI * 4.0)));
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setTargetPosition(e_goal);
        rb.setTargetPosition(e_goal);
        lf.setTargetPosition(e_goal);
        lb.setTargetPosition(e_goal);
        int start_theta = gyro.getHeading(), last_e = 0, cur_e;
        Vector3 drift = new Vector3();
        setDrivePower(power);
        while (!((cur_e = getDrivePosition()) < Math.abs(e_goal) + Lift.thres && cur_e > Math.abs(e_goal) - Lift.thres) && opmode.opModeIsActive()) {
            int de = cur_e - last_e;
            int dtheta = gyro.getHeading() - start_theta;
            drift.x += (Math.PI * (float)de * Math.cos(Math.toRadians(dtheta)))/280f;
            drift.y += (Math.PI * (float)de * Math.sin(Math.toRadians(dtheta)))/280f;
            last_e = cur_e;
            tele("Status", "Driving straight " + Vector3.round(((float)inches)) + " inches...");
            tele("dx", drift.rx());
            telewithup("dy", drift.ry());
        }
        setDrivePower(0);
        int de = getDrivePosition() - last_e;
        int dtheta = gyro.getHeading() - start_theta;
        drift.x += (Math.PI * (float)de * Math.cos(Math.toRadians(dtheta)))/280f;
        drift.y += (Math.PI * (float)de * Math.sin(Math.toRadians(dtheta)))/280f;
        drift.x -= inches;
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return drift;
    }

    /** STRAFE DISTANCE */
    private void strafeDistance (LinearOpMode opmode, double inches, boolean right) {
        if (!opmode.opModeIsActive()) return;

        // Set local variables
        if (inches < 0) { right = !right; inches = Math.abs(inches); }
        int
                right_mult = right ? -1 : 1,
                e_goal = (int)(inches * (1120.0 / strafing_inches_per_rev));
        double power = drive_distance_acceleration;

        // Setup motors
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setTargetPosition(e_goal*right_mult);
        rb.setTargetPosition(-e_goal*right_mult);
        lf.setTargetPosition(-e_goal*right_mult);
        lb.setTargetPosition(e_goal*right_mult);
        setDrivePower(power);

        // Start timer for acceleration and begin loop
        ElapsedTime time = new ElapsedTime();
        while (!checkDrivePosition() && opmode.opModeIsActive()) {
            // Update drive power
            if ((power = 1000.0 * drive_distance_acceleration * time.seconds()) > .5) power = .5;
            setDrivePower(power);

            // Debug
            telewithup("Status", "Strafing " + Vector3.round(((float)inches)) + " inches...");
        }

        // Set motors so they can be used again
        setDrivePower(0);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    private Vector3 strafeDistanceDrift (LinearOpMode opmode, double inches, boolean right) {
        if (!opmode.opModeIsActive()) return new Vector3();
        int right_mult = right ? -1 : 1, e_goal = (int)(inches * (1120.0 / strafing_inches_per_rev));
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setTargetPosition(e_goal*right_mult);
        rb.setTargetPosition(-e_goal*right_mult);
        lf.setTargetPosition(-e_goal*right_mult);
        lb.setTargetPosition(e_goal*right_mult);
        int start_theta = gyro.getHeading(), last_e = 0, cur_e;
        Vector3 drift = new Vector3();
        double power = drive_distance_acceleration;
        setDrivePower(power);
        ElapsedTime time = new ElapsedTime();
        while (!((cur_e = getDrivePosition()) < (e_goal + Lift.thres) && (cur_e > (e_goal - Lift.thres))) && opmode.opModeIsActive()) {
            power = 1000.0 * drive_distance_acceleration * time.seconds();
            if (power > .5) power = .5;
            setDrivePower(power);
            int de = cur_e - last_e;
            int dtheta = gyro.getHeading() - start_theta;
            drift.x += ((float)de * Math.cos(Math.toRadians(dtheta)))/112f;
            drift.y += ((float)de * Math.sin(Math.toRadians(dtheta)))/112f;
            tele("Status", "Strafing " + Vector3.round(((float)inches)) + " inches...");
            tele("dx", drift.rx());
            telewithup("dy", drift.ry());
            last_e = cur_e;
        }
        setDrivePower(0);
        int de = cur_e - last_e;
        int dtheta = gyro.getHeading() - start_theta;
        drift.x += ((float)de * Math.cos(Math.toRadians(dtheta)))/112f;
        drift.y += ((float)de * Math.sin(Math.toRadians(dtheta)))/112f;
        drift.x -= inches;
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return drift;
    }
    private void strafeDistanceAbsDrift (LinearOpMode opmode, double inches, boolean right) {
        if (!opmode.opModeIsActive() || lastKnownPos == null) return;

        // Set local variables
        if (inches < 0) { right = !right; inches = Math.abs(inches); }
        int
                right_mult = right ? -1 : 1,
                e_goal = (int)(inches * (1120.0 / strafing_inches_per_rev)),
                last_e = 0, cur_e, start_theta = (int)lastKnownPos.rotation.z;
        double power = drive_distance_acceleration;

        // Setup motors
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setTargetPosition(e_goal*right_mult);
        rb.setTargetPosition(-e_goal*right_mult);
        lf.setTargetPosition(-e_goal*right_mult);
        lb.setTargetPosition(e_goal*right_mult);
        setDrivePower(power);

        // Start timer for acceleration and begin loop
        ElapsedTime time = new ElapsedTime();
        while (!((cur_e = getDrivePosition()) < (e_goal + Lift.thres) && (cur_e > (e_goal - Lift.thres))) && opmode.opModeIsActive()) {
            // Update drive power
            if ((power = 1000.0 * drive_distance_acceleration * time.seconds()) > .5) power = .5;
            setDrivePower(power);

            // Calculate drift and update position
            int de = cur_e - last_e, dtheta = gyro.getHeading() - start_theta;
            double
                    dx = -right_mult * (de * Math.cos(Math.toRadians(dtheta)))/(1120 / strafing_inches_per_rev),
                    dy = -right_mult * (de * Math.sin(Math.toRadians(dtheta)))/(1120 / strafing_inches_per_rev);
            lastKnownPos.position.x += dx; lastKnownPos.position.y += dy;

            // Debug
            tele("Status", "Strafing " + Vector3.round(((float)inches)) + " inches...");
            lastKnownPos.position.teleout(opmode, "Position");
            teleup();
            verbo("Strafing...");
            verbo("ΔΘ: " + dtheta);
            verbo("x: " + lastKnownPos.position.x);
            verbo("y: " + lastKnownPos.position.y);
            verbo("cur_e: " + cur_e);
            verbo("e_goal: " + e_goal);

            // Update late encoder position
            last_e = cur_e;
        }

        // Stop drive
        setDrivePower(0);

        // Do a final drift calculation
        int de = cur_e - last_e;
        int dtheta = gyro.getHeading() - start_theta;
        lastKnownPos.position.x += -right_mult * (de * Math.cos(Math.toRadians(dtheta)))/(1120 / strafing_inches_per_rev);
        lastKnownPos.position.y += -right_mult * (de * Math.sin(Math.toRadians(dtheta)))/(1120 / strafing_inches_per_rev);

        // Debug
        debug("Done strafing.");
        debug("ΔΘ: " + dtheta);
        debug("x: " + lastKnownPos.position.x);
        debug("y: " + lastKnownPos.position.y);

        // Set motors so they can be used again
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** TURNING */
    private void turnCentrallyToAngle (LinearOpMode opmode, int ang, float power1, float power2) {
        if (!opmode.opModeIsActive()) return;
        if (ang > 180) ang -= 360;

        debug("Goal angle: " + ang);
        debug("Θ: "+specialTheta());

        if (ang < -170 || ang > 170) { // If we are turning to an annoying angle...
            if (ang < 0) ang += 360; // Fix the angle because we are doing this differently.

            debug("Fixed angle 'cause its special.");

            if (theta() > ang) { // Past
                setDriveCWPower(power1); verbo("Set drive power first time.");
                while (theta() > ang - angle_thres && opmode.opModeIsActive()) {
                    tele("Status", "Turning to " + ang + " degrees...");
                    telewithup("Theta", theta());
                    verbo("Θ: "+theta());
                }
                setDriveCCWPower(power2); verbo("Set drive power second time.");
                while (theta() < ang && opmode.opModeIsActive()) {
                    tele("Status", "Turning back to " + ang + " degrees...");
                    telewithup("Theta", theta());
                    verbo("Θ: "+theta());
                }
            } else if (theta() < ang) { // Not past
                setDriveCCWPower(power1); verbo("Set drive power first time.");
                while (theta() < ang + angle_thres && opmode.opModeIsActive()) {
                    tele("Status", "Turning to " + ang + " degrees...");
                    telewithup("Theta", theta());
                    verbo("Θ: "+theta());
                }
                setDriveCWPower(power2); verbo("Set drive power second time.");
                while (theta() > ang && opmode.opModeIsActive()) {
                    tele("Status", "Turning back to " + ang + " degrees...");
                    telewithup("Theta", theta());
                    verbo("Θ: "+theta());
                }
            }
            debug("Done. Θ is " + theta());
        } else if (specialTheta() > ang) { // Otherwise, if we are past our goal...
            setDriveCWPower(power1); verbo("Set drive power first time.");
            while (specialTheta() > ang - angle_thres && opmode.opModeIsActive()) {
                tele("Status", "Turning to " + ang + " degrees...");
                telewithup("Theta", specialTheta());
                verbo("Θ: "+specialTheta());
            }
            setDriveCCWPower(power2); verbo("Set drive power second time.");
            while (specialTheta() < ang && opmode.opModeIsActive()) {
                tele("Status", "Turning back to " + ang + " degrees...");
                telewithup("Theta", specialTheta());
                verbo("Θ: "+specialTheta());
            }
            debug("Done. Θ is " + specialTheta());
        } else if (specialTheta() < ang) { // Lastly, if we aren't turned enough...
            setDriveCCWPower(power1); verbo("Set drive power first time.");
            while (specialTheta() < ang + angle_thres && opmode.opModeIsActive()) {
                tele("Status", "Turning to " + ang + " degrees...");
                telewithup("Theta", specialTheta());
                verbo("Θ: "+specialTheta());
            }
            setDriveCWPower(power2); verbo("Set drive power second time.");
            while (specialTheta() > ang && opmode.opModeIsActive()) {
                tele("Status", "Turning back to " + ang + " degrees...");
                telewithup("Theta", specialTheta());
                verbo("Θ: "+specialTheta());
            }
            debug("Done. Θ is " + specialTheta());
        }
        setDrivePower(0);
    }
    public void turnCentrallyPastAngle(LinearOpMode opmode, int ang, float power) {
        if (!opmode.opModeIsActive()) return;
        if (ang > 180) ang -= 360;

        debug("Goal angle: " + ang);
        debug("Θ: "+specialTheta());

        if (ang < -170 || ang > 170) { // If we are turning to an annoying angle...
            if (ang < 0) ang += 360; // Fix the angle because we are doing this differently.

            debug("Fixed angle 'cause its special.");

            if (theta() > ang) { // Past
                setDriveCWPower(power);
                while (theta() > ang && opmode.opModeIsActive()) {
                    tele("Status", "Turning to " + ang + " degrees...");
                    telewithup("Theta", theta());
                    verbo("Θ: "+theta());
                }
            } else if (theta() < ang) { // Not past
                setDriveCCWPower(power);
                while (theta() < ang && opmode.opModeIsActive()) {
                    tele("Status", "Turning to " + ang + " degrees...");
                    telewithup("Theta", theta());
                    verbo("Θ: "+theta());
                }
            }
            debug("Done. Θ is " + theta());
        } else if (specialTheta() > ang) { // Otherwise, if we are past our goal...
            setDriveCWPower(power);
            while (specialTheta() > ang && opmode.opModeIsActive()) {
                tele("Status", "Turning to " + ang + " degrees...");
                telewithup("Theta", specialTheta());
                verbo("Θ: "+specialTheta());
            }
            debug("Done. Θ is " + specialTheta());
        } else if (specialTheta() < ang) { // Lastly, if we aren't turned enough...
            setDriveCCWPower(power);
            while (specialTheta() < ang && opmode.opModeIsActive()) {
                tele("Status", "Turning to " + ang + " degrees...");
                telewithup("Theta", specialTheta());
                verbo("Θ: "+specialTheta());
            }
            debug("Done. Θ is " + specialTheta());
        }
        setDrivePower(0);
    }
    private void turnBackwardsToAngle (LinearOpMode opmode, int ang, float power) {
        if (!opmode.opModeIsActive()) return;
        int theta = specialTheta();
        if (ang > 180) ang -= 360;

        debug("Goal angle: " + ang);
        debug("Θ: "+specialTheta());

        if (theta < ang) { // CCW
            lf.setPower(-power);
            lb.setPower(-power);
            while ((theta = specialTheta()) < ang && opmode.opModeIsActive()) {
                tele("Status", "Turning to " + ang + " degrees...");
                telewithup("Theta", theta);
                verbo("Θ: "+theta);
            }
        } else if (theta > ang) { // CW
            rf.setPower(-power);
            rb.setPower(-power);
            while ((theta = specialTheta()) > ang && opmode.opModeIsActive()) {
                tele("Status", "Turning to " + ang + " degrees...");
                telewithup("Theta", theta);
                verbo("Θ: "+theta);
            }
        }
        debug("Done. Θ is " + specialTheta());
        setDrivePower(0);
    }
    private void turnForwardsToAngle (LinearOpMode opmode, int ang, float power) {
        if (!opmode.opModeIsActive()) return;
        int theta = specialTheta();
        if (ang > 180) ang -= 360;

        debug("Goal angle: " + ang);
        debug("Θ: "+specialTheta());

        if (theta < ang) { // CCW
            rf.setPower(power);
            rb.setPower(power);
            while ((theta = specialTheta()) < ang && opmode.opModeIsActive()) {
                tele("Status", "Turning to " + ang + " degrees...");
                telewithup("Theta", theta);
                verbo("Θ: "+theta);
            }
        } else if (theta > ang) { // CW
            lf.setPower(power);
            lb.setPower(power);
            while ((theta = specialTheta()) > ang && opmode.opModeIsActive()) {
                tele("Status", "Turning to " + ang + " degrees...");
                telewithup("Theta", theta);
                verbo("Θ: "+theta);
            }
        }
        debug("Done. Θ is " + specialTheta());
        setDrivePower(0);
    }

    /** POSITION CALCULATION */
    private void updatePosition (int start /* theta 1 */) {
        double theta0 = lastKnownPos.rotation.z; int theta2 = specialTheta();
        Vector3 a = f(theta0 + start), b = f(theta0 + theta2);
        lastKnownPos.position.x += b.x - a.x; lastKnownPos.position.y += b.y - a.y;
        debug("Updated position.");
        debug("Changed x by " + (b.x - a.x));
        debug("Changed y by " + (b.y - a.y));
    }
    private static Vector3 f(double delta_theta) { // Mr. Riehm's algorithm
        return new Vector3(
                5.23f*((float)Math.cos(Math.toRadians(39.4 + delta_theta)) - (float)Math.cos(Math.toRadians(39.4))),
                5.23f*((float)Math.sin(Math.toRadians(39.4 + delta_theta)) - (float)Math.sin(Math.toRadians(39.4))),
                0
        );
    }

    /** DRIVE POWER AND ENCODERS */
    private void setDrivePower (double power) {
        rf.setPower(power); rb.setPower(power); lf.setPower(power); lb.setPower(power);
    }
    private void setDriveCCWPower(double power) {
        rf.setPower(power);
        rb.setPower(power);
        lf.setPower(-power);
        lb.setPower(-power);
    }
    private void setDriveCWPower(double power) {
        rf.setPower(-power);
        rb.setPower(-power);
        lf.setPower(power);
        lb.setPower(power);
    }
    private int getDrivePosition () {
        return (Math.abs(rf.getCurrentPosition()) +
                Math.abs(rb.getCurrentPosition()) +
                Math.abs(lf.getCurrentPosition()) +
                Math.abs(lb.getCurrentPosition())) / 4;
    }
    private boolean checkDrivePosition () {
        if (rf.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return false;
        int
                r_rf = Math.abs(rf.getTargetPosition() - rf.getCurrentPosition()),
                r_rb = Math.abs(rb.getTargetPosition() - rb.getCurrentPosition()),
                r_lf = Math.abs(lf.getTargetPosition() - lf.getCurrentPosition()),
                r_lb = Math.abs(lb.getTargetPosition() - lb.getCurrentPosition());
        return ((r_rf+r_rb+r_lf+r_lb)/4) < Lift.thres;
    }

    /** CRYPTOBOX PROCESSING */
    private static double[] sortDividerCoordinates(List<Point> divs) {
        double[] ret = new double[divs.size()];
        for (int i = 0; i < divs.size(); i++) ret[i] = divs.get(i).x;
        for (int i = 1; i < ret.length; i++) {
            int o = 1; double t = ret[i];
            while (i-o>-1 && t<ret[i-o]) { ret[i-o+1] = ret[i-o]; o++; }
            ret[i-o+1] = t;
        }
        return ret;
    }
    private static double[] processDividerCoordinates(List<Point> divs) {
        List<Double> ret = new ArrayList<>();
        double[] toProcess = sortDividerCoordinates(divs);
        for (int i = 0; i < toProcess.length; i++) {
            double t = toProcess[i]; int o = 0;
            while (i + o + 1 < toProcess.length && (t / (o + 1)) + div_threshold >= toProcess[i + o + 1]) {
                t += toProcess[i + o + 1];
                o++;
            }
            t /= o + 1;
            ret.add(t);
            i += o;
        }
        double[] arr = new double[ret.size()];
        for (int i = 0; i < arr.length; i++) arr[i] = ret.get(i);
        return arr;
    }

    /** WAIT */
    private static void wait(LinearOpMode opmode, double seconds) {
        ElapsedTime time = new ElapsedTime();
        while (time.seconds() < seconds) if (!opmode.opModeIsActive()) return;
    }

    /** DEBUG */
    private void tele (String caption, Object data) { opmode.telemetry.addData(caption, data); }
    private void teleup () { opmode.telemetry.update(); }
    private void telewithup (String caption, Object data) {opmode.telemetry.addData(caption, data); opmode.telemetry.update(); }

    private static String verbolastMessage = "";
    private static void verbo (String message) { if (message.equals(verbolastMessage)) return; Log.v(TAG, message); verbolastMessage = message; }
    private static void debug (String message) { Log.d(TAG, message); }
    private static void warn  (String message) { Log.w(TAG, message); }
    private static void error (String message) { Log.e(TAG, message); }

}