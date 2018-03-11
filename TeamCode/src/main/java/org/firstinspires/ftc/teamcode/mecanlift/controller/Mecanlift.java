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
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
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

public class Mecanlift { // ultrasonic sensor is "ultra"

    /** CONSTANTS */
    private static final double                         // Grabber servo positions
            blo = 0.157,                                // Bottom left open
            blc = 0.392,                                // Bottom left close
            bro = 0.863,                                // Bottom right open
            brc = 0.647,                                // Bottom right close
            tlo = 0.863,                                // Top left open
            tlc = 0.627,                                // Top left close
            tro = 0.196,                                // Top right open
            trc = 0.412,                                // Top right close

            jewelArm_down = 0,                          // The jewel servo position for lowering the arm
            jewelArm_up = .793,                         // The jewel servo position for raising the arm

            strafing_inches_per_rev = 10.34,            // Inches strafed for one wheel revolution
            drive_distance_acceleration = 1.0/6000.0,   // Acceleration for driving distances

            div_threshold = 50;                         // Threshold for combining divs in picture

    private static final int
            flip_position = 1000,                       // Position of lift when flipping from ground position

            ccwJewelAngle = 10,                         // Gyro angle to turn to when turning counter-clockwise to hit off jewel
            cwJewelAngle = 350,                         // Gyro angle to turn to when turning clockwise to hit off jewel

            angle_thres = 5;                            // Angle threshold for turning using gyro

    private static final AlignmentCircle
            right = new AlignmentCircle(new Point(641, 1105), 80),
            left = new AlignmentCircle(new Point(378, 1107), 80);

    private static final String
            TAG = "Mecanlift",

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

            pullArmOutMotorN = "out",                   // The name of the motor to pull relic arm out
            pullArmInMotorN = "in",                     // The name of the motor to pull relic arm in
            liftRelicServoN = "up",                     // The name of the servo to lift the relic up
            grabRelicServoN = "grab",                   // The name of the servo to grab the relic

            jewelN = "jewel",                           // The jewel arm servo name
            colorN = "color",                           // The jewel arm color sensor name
            ultrasonicSensorN = "ultra",                // The ultrasonic sensor name
            gyroN = "gyro";                             // The gyro sensor name

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
        opmode = om;
        allianceColor = ac;
        alliancePosition = ap;

        /** DRIVE */
        rf = opmode.hardwareMap.dcMotor.get(drive_rfN);
        rb = opmode.hardwareMap.dcMotor.get(drive_rbN);
        lf = opmode.hardwareMap.dcMotor.get(drive_lfN);
        lb = opmode.hardwareMap.dcMotor.get(drive_lbN);

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);

        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /** LIFT */
        lift = new Lift(opmode.hardwareMap.dcMotor.get(liftN));

        /** GRABBER */
        bl = new ToggleServo(opmode.hardwareMap.servo.get(grabber_blN), blo, blc);
        br = new ToggleServo(opmode.hardwareMap.servo.get(grabber_brN), bro, brc);
        tl = new ToggleServo(opmode.hardwareMap.servo.get(grabber_tlN), tlo, tlc);
        tr = new ToggleServo(opmode.hardwareMap.servo.get(grabber_trN), tro, trc);
        rot = new Rotater(opmode.hardwareMap.dcMotor.get(grabber_rotN), lift);

        /** RELIC ARM */
        initRelicArm();

        /** JEWEL ARM */
        jewelArm = opmode.hardwareMap.servo.get(jewelN);

        /** AUTONOMOUS */
        gyro = opmode.hardwareMap.gyroSensor.get(gyroN);
        if (auto) {
            /** VISUALS */
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

            /** SENSORS */
            jewelSensor = (ModernRoboticsI2cColorSensor)opmode.hardwareMap.colorSensor.get(colorN);
            jewelSensor.resetDeviceConfigurationForOpMode();
            csLightOn();

            ultra = (HiTechnicNxtUltrasonicSensor) opmode.hardwareMap.ultrasonicSensor.get(ultrasonicSensorN);
            ultra.resetDeviceConfigurationForOpMode();

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
        lift.run(lift_pos_tog(), lift_ground(), lift_direct_drive_up(), lift_direct_drive_down(), (opmode.gamepad2.right_stick_button && opmode.gamepad2.left_stick_button));

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
        rot.doRotFix(fix_rotate());

        doRelicArm(-opmode.gamepad2.right_stick_y, opmode.gamepad2.y, opmode.gamepad2.x);
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
            qt_angle = gyro.getHeading() + 180;
            if (qt_angle > 359) qt_angle -= 360;
            powerRF += 1f;
            powerRB += 1f;
            powerLF -= 1f;
            powerLB -= 1f;
        }
    }

    /** RELIC ARM */
//    grab: ng 150, g 80
//    up: d 70, u 215
//    out: continuous
            // This is super lazy programming. Sorry...
    private ContinuousServo out;
    private ToggleServo grab, up;
    private static final double
            grab_no_grab = .588,
            grab_grab = .314,
            up_up = .843,
            up_down = .2745;
    private void initRelicArm() {
//        out = new ContinuousServo(opmode.hardwareMap.servo.get("out"));
//        grab = new ToggleServo(opmode.hardwareMap.servo.get("grab"), grab_no_grab, grab_grab);
//        up = new ToggleServo(opmode.hardwareMap.servo.get("up"), up_down, up_up);
        //out.stop();
    }
    private void startRelicArm() {
//        out.stop();
//        up.open();
//        grab.open();
    }
    private void doRelicArm (float outPower, boolean togGrab, boolean togUp) {
//        out.setPower(outPower);
//        grab.tob(togGrab);
//        up.tob(togUp);
    }
    private void stopRelicArm () { /*out.stop();*/ }

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

//            telewithup("Status", "Driving to key column...");
//            debug("Driving to key column...");
//
//            driveToColumn(opmode);

//            time.reset();
//            VisualsHandler.phoneLightOn();
//            List<Point> divs;
//            Log.d(VisualsHandler.TAG, "---- BEGIN TEST ----");
//            while (opmode.opModeIsActive()) {
//                if (((int)(time.seconds()%3) == 0)) {
//                    divs = visuals.previewCryptobox(allianceColor);
//                    if (!opmode.opModeIsActive()) break;
//                    for (Point com : divs) Log.d(VisualsHandler.TAG, "COM: " + com.x + ", " + com.y);
//                }
//                tele("Status", "Done.");
//                lastKnownPos.position.teleout(opmode, "Pos");
//                lastKnownPos.rotation.teleout(opmode, "Rot");
//                tele("Θ", specialTheta());
//                telewithup("Column", keyColumn());
//            }
//            closeVisuals();
//            return;
        }

        if (opmode.opModeIsActive() && !alliancePosition.justDoJewel() && allianceColor.parkAuto()) {
            debug("PARK ONLY AUTONOMOUS!");
            VisualsHandler.phoneLightOff();
            doPark(opmode);
        }

        VisualsHandler.phoneLightOff();

        debug("Done with autonomous. Time was " + localTime.seconds() + " seconds.");

        lift.groundground();
        lift.waitForEncoders(opmode);

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
        raiseArm();
        br.close();
        bl.close();
        tr.close();
        tl.close();

        visuals.checkJewelsWithCamera();
        JewelConfig post = visuals.jewelConfig;
        debug("Jewel config after initialization: " + post.toString());

        lowerArm();

        calibrateGyro();

        telewithup("Status", "Reading color sensor...");
        Color jewelC = Color.readCS(jewelSensor);
        debug("Jewel color of sensor: " + jewelC.toString());

        if (jewelC == Color.ERROR) {
            telewithup("Status", "Couldn't read it with color sensor...");
            if (post == JewelConfig.ERROR) {
                telewithup("Status", "Couldn't read it actively with camera...");
                if (pre == JewelConfig.ERROR) {
                    telewithup("Status", "Couldn't read it...");
                    raiseArm();
                    return;
                } else turnCCW = allianceColor.turnCCWforJewels(pre);
            } else turnCCW = allianceColor.turnCCWforJewels(post);
        } else turnCCW = allianceColor.turnCCWforJewels(jewelC);

        if (!opmode.opModeIsActive()) return;
        if (turnCCW) {
            telewithup("Status", "Turning left...");
            turnCentrallyPastAngle(opmode, ccwJewelAngle, .5f);
        } else {
            telewithup("Status", "Turning right...");
            turnCentrallyPastAngle(opmode, cwJewelAngle, .5f);
        }
        telewithup("Status", "Raising arm...");
        raiseArm();
        telewithup("Status", "Completed doing jewel.");
        debug("Done doing jewels. Time is " + localTime.seconds() + " seconds.");
    }

    private void driveOffBalance (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        debug("-- DRIVING OFF BALANCING STONE --");

        opmode.hardwareMap.deviceInterfaceModule.get("dim").deregisterForPortReadyCallback(5);
        ElapsedTime localTime = new ElapsedTime();

        lift.setPosition(flip_position);

        turnCentrallyToAngle(opmode, (int)lastKnownPos.rotation.z, .2f, .15f);

        lift.waitForEncoders(opmode);

        if (alliancePosition == Position.CORNER) {

            strafeDistanceAbsDrift(opmode, 27, allianceColor == Color.RED);

            int start = specialTheta();

            turnCentrallyToAngle(opmode, (int)lastKnownPos.rotation.z, .2f, .15f);

            updatePosition(start);
//            telewithup("Status", "Driving two and a half feet...");
//            strafeDistance(opmode, 28, allianceColor != Color.BLUE);
//
//            telewithup("Status", "Straightening up...");
//            turnBackwardsToAngle(opmode, (int) lastKnownPos.rotation.y, .15f);
//
//            telewithup("Status", "Driving back into wall...");
//            driveDistance(opmode, -30, .15f);
//
//            driveDistance(opmode, 23.885, .33f);
//
//            int ang = allianceColor == Color.BLUE ? -90 : 90;
//            turnCentrallyToAngle(opmode, specialTheta() + ang, .33f, .15f);
        } else {

            strafeDistanceAbsDrift(opmode, 34, allianceColor == Color.RED);

            int start = specialTheta();

            turnCentrallyToAngle(opmode, (int)lastKnownPos.rotation.z, .2f, .15f);

            updatePosition(start);
//            telewithup("Status", "Driving three feet...");
//            strafeDistance(opmode, 36, allianceColor != Color.BLUE);
//
//            telewithup("Status", "Straightening up...");
//            turnForwardsToAngle(opmode, (int)lastKnownPos.rotation.y, .15f);
//
//            strafeDistance(opmode, 6.525, allianceColor == Color.BLUE);
        }

        debug("Done driving off balancing stone. Time is " + localTime.seconds() + " seconds.");

//        telewithup("Status", "Opening/closing/raising everything...");
//        br.close();
//        bl.close();
//        tr.close();
//        tl.close();
//        csLightOff();
//        raiseArm(); // Open/close/raise everything before driving off
//
//        telewithup("Status", "Getting straight and raising lift...");
//        lift.setPosition(flip_position); // Start raising the grabber before coming off so the glyph doesn't hit the ground
//        turnCentrallyToAngle(opmode, 0, .33f, .2f); // Straighten up before driving off
//
//        telewithup("Status", "Saving key column...");
//        keyColumn = Column.look(visuals.vuforia); // store key column before coming off
//        if (keyColumn == Column.ERROR) {
//            telewithup("Status", "WARNING: COULD NOT SEE VUMARK!");
//            wait(opmode, 3);
//        }
//
//        telewithup("Status", "Waiting on lift...");
//        lift.waitForEncoders(opmode); // Wait for the lift if its not done moving
//
//        telewithup("Status", "Driving arbitrarily off plate...");
//        driveDistance(opmode, 24, .5f); // Drive an arbitrary two feet off of plate
//
//        telewithup("Status", "Getting position...");
//        VuforiaHandler.PosRot pr = visuals.vuforia.getRelPosWithAverage(3); // Get position and average it for three seconds (for accuracy)
//        pr.position.mmToInches(); // Convert it to inches (reports in mm)
//
//        if (Math.abs(pr.position.z) < z_off_balance) {
//            float d = z_off_balance - Math.abs(pr.position.z);
//            telewithup("Status", "Fixing Z " + Vector3.round(d) + " inches...");
//            if (d < 5) driveDistance(opmode, d, .3f);
//        }
//
//        telewithup("Status", "Aligning to vumark...");
//        turnForwardsToAngle(opmode, (int)lastKnownPos.rotation.y, .15f); // Align to pictograph to get good position
//
//        telewithup("Status", "Getting position...");
//        pr = visuals.vuforia.getRelPosWithAverage(3); // Get position and average it for three seconds (for accuracy)
//        pr.position.mmToInches(); // Convert it to inches (reports in mm)
//
//        int starting_theta = theta(); // Take note of the angle we start at before turning
//
//        if (allianceColor == Color.BLUE) { // Rotate CW if blue
//            int goal_theta = specialTheta() + 270;
//            telewithup("Status", "Turning right to " + goal_theta + " degrees...");
//            turnCentrallyToAngle(opmode, goal_theta, .33f, .2f);
//        }
//
//        if (allianceColor == Color.RED) { // Rotate CCW if red
//            int goal_theta = specialTheta() + 90;
//            telewithup("Status", "Turning left to " + goal_theta + " degrees...");
//            turnCentrallyToAngle(opmode, goal_theta, .33f, .2f);
//        }
//
//        int end_theta = theta(); // Our gyro angle after rotating
//
//        Vector3 phoneD = newCalcPhonePositionDelta((int)lastKnownPos.rotation.y, starting_theta, end_theta); // Calculate phone delta (based off math from Mr. Riehm)
//
//        tele("Status", "Calculated phone delta with difference of " + (end_theta - starting_theta) + " degrees.");
//        phoneD.teleout(opmode, "phone Δ");
//        teleup();
//
//        int ang = allianceColor == Color.BLUE ? -90 : 90;
//        //pr.position.z += phoneD.y; // Add the change in the phones position to our position variable
//        //pr.position.x += phoneD.x;
//        pr.rotation.x = phoneD.x;
//        pr.rotation.z = phoneD.y;
//        pr.position.x = specialTheta() - ang - lastKnownPos.rotation.y;
//
//        //pr.rotation.x = starting_theta;  pr.rotation.z = end_theta; // Store these for later use
//        pr.rotation.y = lastKnownPos.rotation.y;
//        lastKnownPos = pr; // Save the position from before into a variable for use later

    }

    private void driveToCryptobox (LinearOpMode opmode) {
        if (!opmode.opModeIsActive() || lastKnownPos == null) return;

        debug("-- DRIVING TO CRYPTOBOX --");
        ElapsedTime localTime = new ElapsedTime();

        if (alliancePosition == Position.CORNER) {

            driveDistance(opmode, 8, .5f);

            int start = specialTheta();

            turnCentrallyToAngle(opmode, alliancePosition.getAwayFromCryptoAngle(allianceColor, lastKnownPos.position.z), .2f, .15f);

            updatePosition(start);
        } else {
            // TODO: Figure out wtf to do in side positions to take cryptobox picture
        }

        debug("Done driving to cryptobox. Time is " + localTime.seconds() + " seconds.");

//        //VisualsHandler.phoneLightOff();
//
//        // Calculate the distance to drive straight (based on notes)
//        double d = 0;
//        if (allianceColor == Color.RED) d = 17.5 - lastKnownPos.position.x;
//        if (allianceColor == Color.BLUE) d = 37.5 + lastKnownPos.position.x;
//
//        // Drive the distance, watching our angle and calculating drift
//        telewithup("Status", "Driving backwards " + Vector3.round(((float)d)) + " inches...");
//        Vector3 straightDrift = driveDistanceDrift(opmode, -d, .5f);
//
//        // Update our position
//        tele("Status", "Updating position...");
//        straightDrift.teleout(opmode, "Drift");
//        teleup();
//        if (allianceColor == Color.BLUE) {
////            int start = specialTheta();
////            turnBackToSpecialAngle((int)lastKnownPos.rotation.y + 90, .15f);
////            int end = specialTheta();
////            lastKnownPos.position = Vector3.sum(lastKnownPos.position, calcPhonePositionDelta(end-start));
//            lastKnownPos.position.x -= d + straightDrift.x;
//            lastKnownPos.position.z += straightDrift.y;
//        }
//        if (allianceColor == Color.RED) {
////            int start = specialTheta();
////            turnBackToSpecialAngle((int)lastKnownPos.rotation.y + 270, .15f);
////            int end = specialTheta();
////            lastKnownPos.position = Vector3.sum(lastKnownPos.position, calcPhonePositionDelta(end-start));
//            lastKnownPos.position.x += d + straightDrift.x;
//            lastKnownPos.position.z -= straightDrift.y;
//        }
//
////        // Calculate the distance to strafe based on math from notes
////        telewithup("Status", "Calculating strafing distance...");
////        d = keyColumn.inchesToColumnFromVumark(alliancePosition.toID() + allianceColor.toID());
////        boolean right = d < 0;
////        d = Math.abs(lastKnownPos.position.z) - Math.abs(d);
////
////        // Add corrections from testing
////        if (allianceColor == Color.BLUE && alliancePosition == Position.CORNER) {
////            d += 3;
////            if (keyColumn == Column.RIGHT) d -= 8;
////            if (keyColumn == Column.LEFT) d += 6;
////        } else if (allianceColor == Color.RED && alliancePosition == Position.CORNER) {
////            d -= 1;
////            if (keyColumn == Column.LEFT) d -= 8;
////        }
////        if (d < 0) {
////            d = Math.abs(d);
////            right = !right;
////        }
////
////        // If we don't have pos data, let the drivers know and strafe a foot
////        if (lastKnownPos.position.z == 0) {
////            telewithup("WARNING", "There was no Z value. Guessing the distance to drive...");
////            d = 12;
////        }
////
////        // Strafe the distance
////        telewithup("Status", "Strafing " + Vector3.round(((float)d)) + " inches...");
////        Vector3 strafeDrift = strafeDistanceDrift(opmode, d, right);
//
//        // Update our position
//
//        int ang = allianceColor == Color.BLUE ? 270 : 90;
//        // Straighten up again
//        turnForwardsToAngle(opmode, (int)lastKnownPos.rotation.y + ang, .15f);
//
//        telewithup("Status", "Done driving to cryptobox...");
    }

    private void driveToColumn (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        debug("-- DRIVING TO KEY COLUMN --");
        debug("Strafing 8 inches towards cryptobox...");

        ElapsedTime localTime = new ElapsedTime();

        strafeDistance(opmode, 8, allianceColor == Color.BLUE);

        debug("Waiting a second before processing cryptobox...");

        wait(opmode, 1);

        debug("Processing cryptobox...");

        time.reset();
        List<Point> divs = visuals.previewCryptobox(allianceColor);

        debug("Time to process cryptobox is " + time.seconds());
        for (Point div : divs) debug("Got divider: " + div.x + ", " + div.y);
        debug("Based on cryptobox info, I should...");

        switch (divs.size()) {
            case 0: debug("...strafe eight inches towards the box."); break;
            case 1:
                double x = divs.get(0).x, fix = 3;
                if (allianceColor == Color.BLUE) { if (x > 630) fix = 8; else if (x > 480) fix = 6; }
                else if (x < 90) fix = 8; else if (x < 240) fix = 6;
                debug("...strafe " + (int)fix + " inches towards the box."); break;
            case 2:
                double[] xs = sortDividerCoordinates(divs);
                double ppi = Math.abs(xs[1] - xs[0]) / 7.65;
                debug("...do some special stuff. Pixels per inch is " + ppi); break;
            case 3:
                xs = sortDividerCoordinates(divs);
                ppi = Math.abs(xs[2] - xs[0]) / 15.3;
                debug("...do some special stuff. Pixels per inch is " + ppi); break;
            default: debug("...reprocess the box."); break;
        }

        debug("Done driving to key column. Time is " + localTime.seconds() + " seconds.");

    }

    private void alignToCryptobox (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

//        // Calculate and drive distance to drive to line phone up with cryptobox
//        double d = alliancePosition.distanceToAlignCryptobox(allianceColor, lastKnownPos.position);
//        strafeDistance(opmode, Math.abs(d), d < 0);
//
//        // Calculate position based on cryptobox data
//        Mat img = visuals.takeMatPicture();
//        List<Point> cryptobox = VisualsHandler.processCryptobox(img, allianceColor);
//        boolean aligning = true;
//        d = 0;
//        time.reset();
//        do {
//            switch (cryptobox.size()) {
//                case 0:
//                    strafeDistance(opmode, 8, (allianceColor == Color.BLUE) == (alliancePosition == Position.CORNER));
//                    break;
//                case 1:
//                    double x = cryptobox.get(0).x;
//                    double fix = 3;
//                    if (x < img.width() / 8) fix = 8; else if (x < img.width() / 3) fix = 6;
//                    strafeDistance(opmode, fix, (allianceColor == Color.BLUE) == (alliancePosition == Position.CORNER));
//                    break;
//                case 2:
//                    double inchesPerPixel = 7.65 / Math.abs(cryptobox.get(1).x - cryptobox.get(0).x);
//                    d = (allianceColor == Color.BLUE)^(cryptobox.get(0).x<cryptobox.get(1).x) ?
//                            cryptobox.get(1).x :
//                            cryptobox.get(0).x;
//                    aligning = false;
//                    break;
//                case 3:
//
//                    aligning = false;
//                    break;
//                default:
//                    cryptobox = VisualsHandler.processCryptobox(img, allianceColor);
//                    break;
//            }
//        } while (time.seconds() < 5 && aligning && opmode.opModeIsActive());
//
//        // Calculate and drive distance to key column
    }

    private void placeGlyph (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        lift.ground();
        lift.waitForEncoders(opmode);

        driveDistance(opmode, 12, .3f);

        bl.open();
        br.open();
        tl.open();
        tr.open();

        driveDistance(opmode, -4, .3f);

        lift.groundground();
        lift.waitForEncoders(opmode);
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
    private void turnCentrallyPastAngle(LinearOpMode opmode, int ang, float power) {
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
    // TODO: Box coordinates pre-processing

    private static void wait(LinearOpMode opmode, double seconds) {
        ElapsedTime time = new ElapsedTime();
        while (time.seconds() < seconds) if (!opmode.opModeIsActive()) return;
    }

    /** DEBUG */
    private void tele (String caption, Object data) { opmode.telemetry.addData(caption, data); }
    private void teleup () { opmode.telemetry.update(); }
    private void telewithup (String caption, Object data) {opmode.telemetry.addData(caption, data); opmode.telemetry.update(); }

    private static String verbolastMessage = "", debugLastMessage = "", warnLastMessage = "", errorLastMessage = "";
    private static void verbo (String message) { if (message.equals(verbolastMessage)) return; Log.v(TAG, message); verbolastMessage = message; }
    private static void debug (String message) { if (message.equals(debugLastMessage)) return; Log.d(TAG, message); debugLastMessage = message; }
    private static void warn  (String message) { if (message.equals(warnLastMessage )) return; Log.w(TAG, message); warnLastMessage  = message; }
    private static void error (String message) { if (message.equals(errorLastMessage)) return; Log.e(TAG, message); errorLastMessage = message; }

    /** DEPRECATED */
    @Deprecated private static final int
            alignment_x = 455,      // Horizontal position of the vertical alignment line
            alignment_y = 1042;     // Vertical position of the horizontal alignment line
    @Deprecated public void drive(
            boolean liftUp, boolean liftDown, boolean liftDirectUp, boolean liftDirectDown,
            boolean bottomToggle, boolean topToggle, boolean rot, boolean fixRot,
            float straight, float strafe, float rotate, boolean slow, boolean quickR) {
        /** LIFT */
        lift.run(liftUp, liftDown, liftDirectUp, liftDirectDown);

        /** GRABBER */
        if (this.rot.flipped) {
            bl.tob(topToggle);
            br.tob(topToggle);
            tl.tob(bottomToggle);
            tr.tob(bottomToggle);
        } else {
            bl.tob(bottomToggle);
            br.tob(bottomToggle);
            tl.tob(topToggle);
            tr.tob(topToggle);
        }
        this.rot.doRotation(rot);
        this.rot.doRotFix(fixRot);

        /** MECANUM WHEELS */
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
        doQuickTurn(quickR);
        if (powerRF > 1) {powerRF = 1f;} else if (powerRF < -1f) {powerRF = -1f;}
        if (powerRB > 1) {powerRB = 1f;} else if (powerRB < -1f) {powerRB = -1f;}
        if (powerLF > 1) {powerLF = 1f;} else if (powerLF < -1f) {powerLF = -1f;}
        if (powerLB > 1) {powerLB = 1f;} else if (powerLB < -1f) {powerLB = -1f;}
        if (slow) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }
        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);
    }
    @Deprecated public void drive(
            boolean liftUp, boolean liftDown, boolean liftDirectUp, boolean liftDirectDown,
            float brPos, float blPos, float trPos, float tlPos, boolean rot, boolean fixRot,
            float straight, float strafe, float rotate, boolean slow, boolean quickR) {
        /** LIFT */
        lift.run(liftUp, liftDown, liftDirectUp, liftDirectDown);

        // TODO: Make it so that when flipped, the servos keep each of their positions until change of the float
        /** GRABBER */
        bl.setPos(blPos);
        br.setPos(brPos);
        tl.setPos(tlPos);
        tr.setPos(trPos);
        this.rot.doRotation(rot);
        this.rot.doRotFix(fixRot);

        /** MECANUM WHEELS */
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
        doQuickTurn(quickR);
        if (powerRF > 1) {powerRF = 1f;} else if (powerRF < -1) {powerRF = -1f;}
        if (powerRB > 1) {powerRB = 1f;} else if (powerRB < -1) {powerRB = -1f;}
        if (powerLF > 1) {powerLF = 1f;} else if (powerLF < -1) {powerLF = -1f;}
        if (powerLB > 1) {powerLB = 1f;} else if (powerLB < -1) {powerLB = -1f;}
        if (slow) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }
        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);
    }
    @Deprecated public Mecanlift (OpMode om, Color allianceColor) { init(om, true, allianceColor, Position.ERROR); }
    @Deprecated private void doJewels () {
        telewithup("Status", "Opening/closing/lowering everything...");
        br.close();
        bl.close();
        tr.open();
        tl.open();
        lowerArm();

        calibrateGyro();

        telewithup("Status", "Reading color sensor...");
        Color jewelC = Color.readCS(jewelSensor);

        if (jewelC == Color.ERROR) { raiseArm(); telewithup("Status", "Couldn't read it..."); return; }

        if (allianceColor.turnCCWforJewels(jewelC)) {
            telewithup("Status", "Turning left...");
            turnPastAngle(ccwJewelAngle, true, .5f);
        } else {
            telewithup("Status", "Turning right...");
            turnPastAngle(cwJewelAngle, false, .5f);
        }
        telewithup("Status", "Raising arm...");
        raiseArm();
        telewithup("Status", "Completed doing jewel.");
    }
    @Deprecated private void wait(double seconds) {
        ElapsedTime time = new ElapsedTime();
        while (time.seconds() < seconds) ;
    }
    @Deprecated public void tellDriverRelPos() { visuals.vuforia.tellDriverRelPos(); }
    @Deprecated private int driveCurPosition () {
        return (rf.getCurrentPosition() + rb.getCurrentPosition() + lf.getCurrentPosition() + lb.getCurrentPosition()) / 4;
    }
    @Deprecated private int driveNewCurPosition () {
        return (Math.abs(rf.getCurrentPosition()) +
                Math.abs(rb.getCurrentPosition()) +
                Math.abs(lf.getCurrentPosition()) +
                Math.abs(lb.getCurrentPosition())) / 4;
    }
    @Deprecated private static Vector3 calcPhonePositionDelta(float delta_theta) {
        return new Vector3(
                5.23f*((float)Math.cos(Math.toRadians(39.4 + delta_theta)) - .7727f),
                0,
                5.23f*(.6347f - (float)Math.sin(Math.toRadians(39.4 + delta_theta)))
        );
    }
    @Deprecated private static Vector3 newCalcPhonePositionDelta(int theta0, int theta1, int theta2) {
        Vector3 a = f(theta1-theta0), b = f(theta2-theta0);
        return new Vector3((a.x*-1)+b.x, (a.y*-1)+b.y, 0);
    }
    @Deprecated private void doParkAutonomous (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        telewithup("Status", "Doing jewels...");
        doJewels(opmode);

        if (alliancePosition.justDoJewel()) return;

        telewithup("Status", "Parking...");
        doPark(opmode);

        VisualsHandler.phoneLightOff();

        while (opmode.opModeIsActive()) {
            tele("Status", "Done.");
            lastKnownPos.teleout(opmode);
            tele("Θ", specialTheta());
            telewithup("Column", keyColumn());
        }

        closeVisuals();
    }
    @Deprecated private void doSideToCryptobox (LinearOpMode opmode) {
        if (!opmode.opModeIsActive() || lastKnownPos == null) return;
        VisualsHandler.phoneLightOff();

        lift.setPosition(flip_position);

        turnCentrallyToAngle(opmode, (int) lastKnownPos.rotation.y, .33f, .2f);

        lift.waitForEncoders(opmode);

        // Drive three feet
        telewithup("Status", "Driving three feet...");
        Vector3 strafeDrift = strafeDistanceDrift(opmode, 36, allianceColor != Color.BLUE);

        lastKnownPos.position.mmToInches();

        telewithup("Status", "Updating position...");
        lastKnownPos.position.z += strafeDrift.y;
        if (allianceColor == Color.BLUE) {
            lastKnownPos.position.x -= 36 + strafeDrift.x;
        } else if (allianceColor == Color.RED) lastKnownPos.position.x += 36 + strafeDrift.x;

        turnForwardsToAngle(opmode, (int) lastKnownPos.rotation.y, .15f);

        //strafeDistance(opmode, 2, allianceColor == Color.BLUE); // HERE

//        turnCentrallyToAngle(opmode, (int)lastKnownPos.rotation.y + 180, .33f, .2f);
//
//        double d = keyColumn.inchesToColumnFromVumark(alliancePosition.toID() + allianceColor.toID());
//        boolean right = d > 0;
//        d = Math.abs(d) /*- Math.abs(lastKnownPos.position.x)*/;
//
//        telewithup("Status", "Strafing " + Vector3.round(((float)d)) + " inches...");
//        strafeDrift = strafeDistanceDrift(opmode, d, right);

        telewithup("Status", "Done driving to cryptobox from side...");
    }
    @Deprecated private void turnPastAngle(int ang, boolean ccw, float speed) {
        if (ccw) {
            rf.setPower(speed);
            rb.setPower(speed);
            lf.setPower(-speed);
            lb.setPower(-speed);
            while (gyro.getHeading() > ang);
            while (gyro.getHeading() < ang);
        } else {
            rf.setPower(-speed);
            rb.setPower(-speed);
            lf.setPower(speed);
            lb.setPower(speed);
            while (gyro.getHeading() < ang);
            while (gyro.getHeading() > ang);
        }
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
    }
    @Deprecated public void turnToAngle(int ang, boolean ccw, float speed) {
        if (ccw) {
            rf.setPower(speed);
            rb.setPower(speed);
            lf.setPower(-speed);
            lb.setPower(-speed);
            while (gyro.getHeading() > ang);
            while (gyro.getHeading() < ang);
        } else {
            rf.setPower(-speed);
            rb.setPower(-speed);
            lf.setPower(speed);
            lb.setPower(speed);
            while (gyro.getHeading() < ang);
            while (gyro.getHeading() > ang);
        }
        setDrivePower(0);
    }
    @Deprecated private void turnToSpecialAngle(int ang, float power) {
//        ElapsedTime time = new ElapsedTime();
//        while (time.seconds() < 3) {
//            telewithup("CCW, theta", specialTheta());
//        }
        int theta = specialTheta();
        if (ang > 180) ang -= 360;
        if (theta < ang) { // CCW
            rf.setPower(power);
            rb.setPower(power);
            while ((theta = specialTheta()) < ang) telewithup("CCW, theta", theta);
            //setDriveCWPower(power / 2f);
            //while ((theta = specialTheta()) > ang) telewithup("CW, theta", theta);
        } else if (theta > ang) { // CW
            lf.setPower(power);
            lb.setPower(power);
            while ((theta = specialTheta()) > ang) telewithup("CW, theta", theta);
            //setDriveCCWPower(power / 2f);
            //while ((theta = specialTheta()) < ang) telewithup("CCW, theta", theta);
        }
        setDrivePower(0);
    }
    @Deprecated public void turnBackToSpecialAngle(int ang, float power) {
//        ElapsedTime time = new ElapsedTime();
//        while (time.seconds() < 3) {
//            telewithup("CCW, theta", specialTheta());
//        }
        int theta = specialTheta();
        if (ang > 180) ang -= 360;
        if (theta < ang) { // CCW
            lf.setPower(-power);
            lb.setPower(-power);
            while ((theta = specialTheta()) < ang) telewithup("CCW, theta", theta);
            //setDriveCWPower(power / 2f);
            //while ((theta = specialTheta()) > ang) telewithup("CW, theta", theta);
        } else if (theta > ang) { // CW
            rf.setPower(-power);
            rb.setPower(-power);
            while ((theta = specialTheta()) > ang) telewithup("CW, theta", theta);
            //setDriveCCWPower(power / 2f);
            //while ((theta = specialTheta()) < ang) telewithup("CCW, theta", theta);
        }
        setDrivePower(0);
    }
    @Deprecated private void getStraight() {
        int theta = specialTheta();
        if (theta > 0) { // Turn CW
            setDriveCWPower(.33);
            while ((theta = specialTheta()) > 0) {
                tele("Status", "Getting straight... Pass 1");
                telewithup("CW, theta", theta);
            }
            setDriveCCWPower(.2);
            while ((theta = specialTheta()) < 0) {
                tele("Status", "Getting straight... Pass 2");
                telewithup("CCW, theta", theta);
            }
        } else if (theta < 0) { // Turn CCW
            setDriveCCWPower(.33);
            while ((theta = specialTheta()) < 0) {
                tele("Status", "Getting straight... Pass 1");
                telewithup("CCW, theta", theta);
            }
            setDriveCWPower(.2);
            while ((theta = specialTheta()) > 0) {
                tele("Status", "Getting straight... Pass 2");
                telewithup("CW, theta", theta);
            }
        }
        setDrivePower(0);
        telewithup("Status", "Straightened.");
    }
    @Deprecated private void getParallel() { // To vumark

        getPerpendicular();

//        int theta = theta();
//        float goal = lastKnownPos.rotation.y + 180;
//        if (goal > 180) goal -= 360;
//        if (theta > goal) { // Turn CW
//            setDriveCWPower(.33);
//            while ((theta = theta()) > goal) {
//                tele("Status", "Getting parallel... Pass 1");
//                tele("Goal", goal);
//                telewithup("CW, theta", theta);
//            }
//            setDriveCCWPower(.2);
//            while ((theta = theta()) < goal) {
//                tele("Status", "Getting parallel... Pass 2");
//                tele("Goal", goal);
//                telewithup("CCW, theta", theta);
//            }
//        } else if (theta < goal) { // Turn CCW
//            setDriveCCWPower(.33);
//            while ((theta = theta()) < goal) {
//                tele("Status", "Getting parallel... Pass 1");
//                tele("Goal", goal);
//                telewithup("CCW, theta", theta);
//            }
//            setDriveCWPower(.2);
//            while ((theta = theta()) > goal) {
//                tele("Status", "Getting parallel... Pass 2");
//                tele("Goal", goal);
//                telewithup("CW, theta", theta);
//            }
//        }

        int theta = specialTheta(), goal = theta + 180;
        setDriveCCWPower(.3);
        while (theta < goal) {
            tele("Status", "Getting parallel... Pass 1");
            tele("Goal", goal);
            telewithup("CCW, theta", theta);
            if ((theta = theta()) > 270) theta = specialTheta();
        }
        setDriveCWPower(.2);
        while ((theta = theta()) > goal) {
            tele("Status", "Getting parallel... Pass 2");
            tele("Goal", goal);
            telewithup("CW, theta", theta);
        }
//        if (theta > goal) { // Turn CW
//            setDriveCWPower(.33);
//            while ((theta = theta()) > goal) {
//                tele("Status", "Getting parallel... Pass 1");
//                tele("Goal", goal);
//                telewithup("CW, theta", theta);
//            }
//            setDriveCCWPower(.2);
//            while ((theta = theta()) < goal) {
//                tele("Status", "Getting parallel... Pass 2");
//                tele("Goal", goal);
//                telewithup("CCW, theta", theta);
//            }
//        } else if (theta < goal) { // Turn CCW
//            setDriveCCWPower(.33);
//            while ((theta = theta()) < goal) {
//                tele("Status", "Getting parallel... Pass 1");
//                tele("Goal", goal);
//                telewithup("CCW, theta", theta);
//            }
//            setDriveCWPower(.2);
//            while ((theta = theta()) > goal) {
//                tele("Status", "Getting parallel... Pass 2");
//                tele("Goal", goal);
//                telewithup("CW, theta", theta);
//            }
//        }
        setDrivePower(0);
        telewithup("Status", "Paralleled.");
    }
    @Deprecated private void getPerpendicular() { // To vumark
        int theta = specialTheta();
        float goal = lastKnownPos.rotation.y;
        if (theta > goal) { // Turn CW
            setDriveCWPower(.33);
            while ((theta = specialTheta()) > goal) {
                tele("Status", "Getting parallel... Pass 1");
                telewithup("CW, theta", theta);
            }
            setDriveCCWPower(.2);
            while ((theta = specialTheta()) < goal) {
                tele("Status", "Getting parallel... Pass 2");
                telewithup("CCW, theta", theta);
            }
        } else if (theta < goal) { // Turn CCW
            setDriveCCWPower(.33);
            while ((theta = specialTheta()) < goal) {
                tele("Status", "Getting parallel... Pass 1");
                telewithup("CCW, theta", theta);
            }
            setDriveCWPower(.2);
            while ((theta = specialTheta()) > goal) {
                tele("Status", "Getting parallel... Pass 2");
                telewithup("CW, theta", theta);
            }
        }
        setDrivePower(0);
        telewithup("Status", "Paralleled.");
    }
    @Deprecated public void strafeDistance (int count, boolean right) {
        int right_mult = right ? -1 : 1;
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setTargetPosition(count*right_mult);
        rb.setTargetPosition(-count*right_mult);
        lf.setTargetPosition(-count*right_mult);
        lb.setTargetPosition(count*right_mult);
        setDrivePower(.5);
        while (!checkDrivePosition()) ;
        setDrivePower(0);
    }
    @Deprecated public void strafeAccelDistance (int count, double pPerMS, boolean right) {
        int right_mult = right ? -1 : 1;
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setTargetPosition(count*right_mult);
        rb.setTargetPosition(-count*right_mult);
        lf.setTargetPosition(-count*right_mult);
        lb.setTargetPosition(count*right_mult);
        double power = pPerMS;
        setDrivePower(power);
        ElapsedTime time = new ElapsedTime();
        while (!checkDrivePosition()) {
            power = 1000.0 * pPerMS * time.seconds();
            if (power > .5) power = .5;
            setDrivePower(power);
            opmode.telemetry.addData("power", power);
            opmode.telemetry.addData("time", time.seconds());
            opmode.telemetry.update();
        }
        setDrivePower(0);
    }
    @Deprecated public Vector3 strafeDistanceDrift (double inches, boolean right) {
        double inchesPerRev = 10.34, accel = 1.0/6000.0;
        int right_mult = right ? -1 : 1, e_goal = (int)(inches * (1120 / inchesPerRev));
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
        double power = accel;
        setDrivePower(power);
        ElapsedTime time = new ElapsedTime();
        while (!((cur_e = driveNewCurPosition()) < (e_goal + Lift.thres) && (cur_e > (e_goal - Lift.thres)))) {
            power = 1000.0 * accel * time.seconds();
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
    @Deprecated private void driveDistance (double inches) {
        int encoder = 1120 * (int)(inches/(Math.PI * 4.0));
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
        setDrivePower(.5);
        while (!Lift.update_encoders(rb));
        rf.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Deprecated private void driveDistance (double inches, float power) {
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
        while (!Lift.update_encoders(rb));
        rf.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Deprecated public Vector3 driveDistanceDrift (double inches) {
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
        setDrivePower(.5);
        while (!((cur_e = driveCurPosition()) < e_goal + Lift.thres && cur_e > e_goal - Lift.thres)) {
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
        int de = driveCurPosition() - last_e;
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
    @Deprecated public void driveToBox (FIELDPOS pos) {
        lift.lift();
        lift.waitForEncoders();
        driveDistance(pos.inchesToBox);
        lift.groundground();
        lift.waitForEncoders();
    }
    @Deprecated public enum FIELDPOS {
        BLUE_SIDE(70,true,36), BLUE_CORNER(64,true,32.8), RED_SIDE(290,false,36), RED_CORNER(296,false,32.8);
        public final int turnAngle;
        public final boolean ccw;
        public final double inchesToBox;
        FIELDPOS(int ta, boolean ccw, double in) { turnAngle = ta; this.ccw = ccw; inchesToBox = in; }
    }
    @Deprecated public void turnToBox (FIELDPOS pos) {
        turnPastAngle(pos.turnAngle, pos.ccw, .5f);
//        int ang = pos.turnAngle;
//        if (pos.ccw) {
//            rf.setPower(.5);
//            rb.setPower(.5);
//            lf.setPower(-.5);
//            lb.setPower(-.5);
//        } else {
//            rf.setPower(-.5);
//            rb.setPower(-.5);
//            lf.setPower(.5);
//            lb.setPower(.5);
//        }
//        while (!checkGyro(ang));
//        rf.setPower(0);
//        rb.setPower(0);
//        lf.setPower(0);
//        lb.setPower(0);
    }

    /** OLD CODE */
//    private void doRotation (boolean b) { // NOTE: Moved to separate class
//        if (rotating) {
//            if (Lift.update_encoders(rot)) { // Done rotating
//                rotating = false;
//                flipped = !flipped;
//                if (lifted) { lift.ground(); lifted = false; }
//                return;
//            } else return;
//        }
//        if (rotated && !b) { rotated = false; }
//        if (!b) return;
//        if (!rotated) { // Start rotating
//            if (lift.grounded()) { lifted = true; lift.setPosition(flip_position); }
//            if (flipped) {
//                rot.setTargetPosition(nflipped_pos);
//                rot.setPower(flip_power);
//            } else {
//                rot.setTargetPosition(flipped_pos);
//                rot.setPower(flip_power);
//            }
//            rotating = true;
//        }
//    }
//    private void doRotFix (boolean b) { NOTE: Moved to separate class
//        if (fixing && Lift.update_encoders(rot)) {
//            rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            if (lifted) { lift.ground(); lifted = false; }
//            fixing = false;
//            return;
//        }
//        if (fixed && !b) { fixed = false; return; }
//        if (!b) return;
//        if (!fixed) {
//            if (lift.grounded()) { lifted = true; lift.setPosition(flip_position); }
//            rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rot.setTargetPosition(-flipped_pos);
//            rot.setPower(-flip_power);
//            fixing = true;
//        }
//    }
//    private boolean checkGyro () { NOTE: Didn't work. It was too specific that the robot just passed right over it.
//        return (gyro.getHeading() < qt_angle + 2) && (gyro.getHeading() > qt_angle - 2);
//    }
//
//    private boolean checkGyro (int goal) { NOTE: Didn't work. See above.
//        return (gyro.getHeading() < goal + 2) && (gyro.getHeading() > goal - 2);
//    }
//
//    private void checkRotation () { NOTE: Replaced by driver initiated fixing. Easier to program that way.
//        if (!checking) {
//            if (flipped) {
//                rot.setTargetPosition(nflipped_pos);
//                rot.setPower(-1);
//            } else {
//                rot.setTargetPosition(flipped_pos);
//                rot.setPower(1);
//            }
//            time = new ElapsedTime();
//            checking = true;
//            return;
//        }
//        if (time.seconds() >= 1) {
//            if (!flipped && rot.getCurrentPosition() < 10) {
//                flipped = true;
//            } else {
//
//            }
//        }
//    }

}