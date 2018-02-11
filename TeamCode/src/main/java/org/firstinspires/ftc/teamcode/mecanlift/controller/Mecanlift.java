package org.firstinspires.ftc.teamcode.mecanlift.controller;

// Created on 1/28/2018 at 11:29 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.visuals.Vector3;
import org.firstinspires.ftc.teamcode.visuals.VisualsHandler;
import org.firstinspires.ftc.teamcode.visuals.VuforiaHandler;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import static org.firstinspires.ftc.teamcode.visuals.Vector3.round;

public class Mecanlift {

    /** ENUMS */
    /** COLOR ENUM FOR JEWELS */
    public enum Color {
        RED, BLUE, ERROR;
        public static Color readCS (ColorSensor s) {
            if (s.red() > s.blue()) return RED;
            if (s.blue() > s.red()) return BLUE;
            return ERROR;
        }
        public boolean turnCCW (Color jewelColor) {
            switch (this) {
                case RED: return jewelColor == BLUE;
                case BLUE: return jewelColor == RED;
                default: return false;
            }
        }
        public boolean isBoxToLeft () {
            switch (this) {
                case RED: return false;
                case BLUE: return true;
                default: return false;
            }
        }
        public int toID () {
            switch (this) {
                case RED: return red_id;
                case BLUE: return blue_id;
                default: return 0;
            }
        }
    }

    /** POSITION ENUM FOR PARKING */
    public enum Position {
        SIDE, CORNER, JEWEL_ONLY, ERROR;
        public boolean justDoJewel() { return this == JEWEL_ONLY; }
        public int toID () {
            switch (this) {
                case SIDE: return side_id;
                case CORNER: return corner_id;
                default: return 0;
            }
        }
        public int getParkingAngle (Color allianceColor) {
            switch (this.toID() + allianceColor.toID()) {
                case corner_id + red_id: return rcp_turn_angle;
                case side_id + red_id: return rsp_turn_angle;
                case corner_id + blue_id: return bcp_turn_angle;
                case side_id + blue_id: return bsp_turn_angle;
                default: return 0;
            }
        }
        public double getParkingDistance () {
            switch (this) {
                case SIDE: return sp_distance;
                case CORNER: return cp_distance;
                default: return 0;
            }
        }
    }

    /** KEY COLUMN ENUM */
    public enum Column {
        LEFT, RIGHT, CENTER, ERROR;
        public static Column look(VuforiaHandler v) {
            RelicRecoveryVuMark r = v.lookingAtMark();
            if (r == null) return ERROR;
            switch (r) {
                case LEFT: return LEFT;
                case RIGHT: return RIGHT;
                case CENTER: return CENTER;
                case UNKNOWN: return ERROR;
                default: return ERROR;
            }
        }
        public double inchesToColumnFromVumark (int alliancePositionID) { // NOTE: return a v3 with the specific axis of distance changed
            if (this == ERROR) return 0;
            float ret = box_start_inches;
            float column_adjustment = 0;
            switch (this) {
                case LEFT:  column_adjustment += column_adjustment_inches;
                case RIGHT: column_adjustment -= column_adjustment_inches;
            }
            switch (alliancePositionID) {
                case corner_id + red_id:    return ret+column_adjustment;
                case side_id + red_id:      ret -= mark_offset_inches; return -ret-column_adjustment;
                case corner_id + blue_id:   return column_adjustment-ret;
                case side_id + blue_id:     ret += mark_offset_inches; return ret-column_adjustment;
                default:                    return 0;
            }
        }
        public String toString() {
            switch (this) {
                case LEFT:      return "LEFT";
                case RIGHT:     return "RIGHT";
                case CENTER:    return "CENTER";
                default:        return "ERROR";
            }
        }
    }

    @Deprecated public enum FIELDPOS {
        BLUE_SIDE(70,true,36), BLUE_CORNER(64,true,32.8), RED_SIDE(290,false,36), RED_CORNER(296,false,32.8);
        public final int turnAngle;
        public final boolean ccw;
        public final double inchesToBox;
        FIELDPOS(int ta, boolean ccw, double in) { turnAngle = ta; this.ccw = ccw; inchesToBox = in; }
    }

    /** CONSTANTS */
    private static final double     // Grabber servo positions
            blo = 0.078,            // Bottom left open
            blc = 0.310,            // Bottom left close
            bro = 0.98,             // Bottom right open
            brc = 0.670,            // Bottom right close
            tlo = 0.98,             // Top left open
            tlc = 0.700,            // Top left close
            tro = 0.118,            // Top right open
            trc = 0.450,            // Top right close

            jewelArm_down = .2,    // The jewel servo position for lowering the arm
            jewelArm_up = 1,        // The jewel servo position for raising the arm

            flip_power = .5,        // Power of the flipping motor
    // TODO: Add two movements to flipping action, one fast and long, the other short but slow

            sp_distance = 36.0,     // Distance in inches to drive in order to park from side position
            cp_distance = 32.8;     // Distance in inches to drive in order to park from corner position

    private static final int
            nflipped_pos = 0,       // Position of rotational motor when not flipped
            flipped_pos = 5100,     // Position of rotational motor when flipped
            flip_position = 1000,   // Position of lift when flipping from ground position

            alignment_x = 455,      // Horizontal position of the vertical alignment line
            alignment_y = 1042,     // Vertical position of the horizontal alignment line

            circle_radius = 80,     // Alignment circle radius

            ccwAngle = 10,          // Gyro angle to turn to when turning counter-clockwise to hit off jewel
            cwAngle = 350,          // Gyro angle to turn to when turning clockwise to hit off jewel

            // NOTE: corner refers to the position farthest away from the relic zones, side the closest.
            rcp_turn_angle = 296,   // Gyro angle to turn to park when the robot's stating position is the red corner
            rsp_turn_angle = 290,   // Gyro angle to turn to park when the robot's stating position is the red side
            bcp_turn_angle = 64,    // Gyro angle to turn to park when the robot's stating position is the blue corner
            bsp_turn_angle = 70,    // Gyro angle to turn to park when the robot's stating position is the blue side

            corner_id = 1,          // Number ID for corner position
            side_id = 4,            // Number ID for side position
            red_id = 2,             // Number ID for red alliance color
            blue_id = 7;            // Number ID for blue alliance color

    private static final Point
            red_center = new Point(621, 1095),
            blue_center = new Point(378, 1107);

    private static final float      // Based off notes from 020618
            box_start_inches = 36f,
            mark_offset_inches = 10.5f,
            column_adjustment_inches = 7.63f;

    private static final String
            drive_rfN = "rf",       // The front right motor name
            drive_rbN = "rb",       // The back right motor name
            drive_lfN = "lf",       // The front left motor name
            drive_lbN = "lb",       // The back left motor name

            // Note: Looking at robot, not from
            grabber_blN = "bl",     // The bottom left grabber servo name
            grabber_brN = "br",     // The bottom right grabber servo name
            grabber_tlN = "tl",     // The top left grabber servo name
            grabber_trN = "tr",     // The top right grabber servo name
            grabber_rotN = "rot",   // The rotational motor name

            liftN = "lift",         // The lift motor name

            jewelN = "jewel",       // The jewel arm servo name
            colorN = "color",       // The jewel arm color sensor name
            gyroN = "gyro";         // The gyro sensor name

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
    private DcMotor rot;
    private boolean
            flipped = false,    // Boolean to determine whether the grabber is flipped or not
            rotated = false,    // Boolean for rotating button logic
            rotating = false,   // Boolean to determine whether or not we are currently rotating the grabber
            fixing = false,     // Boolean to determine whether or not we are currently fixing the grabber
            fixed = false,      // Boolean for fixing button logic
            lifted = false;     // Boolean to determine whether or not the lift has gone up a little bit for rotating

    /** LIFT */
    private Lift lift;

    /** VISUALS */
    private VisualsHandler visuals;
    private ElapsedTime time;

    /** SENSORS */
    private GyroSensor gyro; //NOTE: counter clockwise is positive for gyro
    private ModernRoboticsI2cColorSensor jewelSensor;

    /** JEWELS */
    private Servo jewelArm;

    /** FOR AUTONOMOUS USE */
    private Color allianceColor;
    private Position alliancePosition;
    private Column keyColumn = Column.ERROR;
    private VuforiaHandler.PosRot lastKnownPos;

    /** INITIALIZERS */
    public Mecanlift (OpMode om) { init(om, false, Color.ERROR, Position.ERROR); } // USE IF TELEOP
    public Mecanlift (OpMode om, Color allianceColor, Position alliancePosition) { init(om, true, allianceColor, alliancePosition); }
    private void init(OpMode om, boolean auto, Color ac, Position ap) {
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
        rot = opmode.hardwareMap.dcMotor.get(grabber_rotN);
        rot.setDirection(DcMotorSimple.Direction.REVERSE);
        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /** JEWEL ARM */
        jewelArm = opmode.hardwareMap.servo.get(jewelN);

        /** AUTONOMOUS */
        gyro = opmode.hardwareMap.gyroSensor.get(gyroN);
        if (auto) {
            /** VISUALS */
            visuals = new VisualsHandler(opmode, false);
            visuals.vertx = alignment_x;
            visuals.hory = alignment_y;
            visuals.red.center = red_center;
            visuals.red.radius = circle_radius;
            visuals.blue.center = blue_center;
            visuals.blue.radius = circle_radius;
            VisualsHandler.phoneLightOn();
            visuals.showAlignmentCircles();
            time = new ElapsedTime();

            /** SENSORS */
            jewelSensor = (ModernRoboticsI2cColorSensor)opmode.hardwareMap.colorSensor.get(colorN);
            jewelSensor.enableLed(true);

        } else { opmode.hardwareMap.colorSensor.get(colorN).enableLed(false); }
    }

    public void calibrateGyro () {
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            opmode.telemetry.addData("Status", "Waiting on gyro...");
            opmode.telemetry.update();
        }
    }
    public int theta() { return gyro.getHeading(); }

    @Deprecated public Mecanlift (OpMode om, /*boolean auto,*/ Color allianceColor) { init(om, true, allianceColor, Position.ERROR); }

    /** INIT LOOP METHODS */
    public void showAligning () { if (((int)(time.seconds()%2) == 1)) visuals.showAlignmentCircles(); }

    /** START METHODS */
    public void start () { raiseArm(); bl.open(); br.open(); tl.open(); tr.open(); lift.start(); }

    /** LOOP METHODS */
    public void drive() { // Assuming default controls as of 1/28

        /** LIFT */
        lift.run(opmode.gamepad1.dpad_up, opmode.gamepad1.dpad_down, opmode.gamepad1.y, opmode.gamepad1.x);

        /** GRABBER */
        boolean a = opmode.gamepad1.a, b = opmode.gamepad1.b;
        if (!flipped) {
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
        doRotation(opmode.gamepad1.right_stick_button);
        doRotFix(opmode.gamepad1.right_bumper && opmode.gamepad1.left_bumper);

        /** MECANUM WHEELS */
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
    public void drive(
            boolean liftUp, boolean liftDown, boolean liftDirectUp, boolean liftDirectDown,
            boolean bottomToggle, boolean topToggle, boolean rot, boolean fixRot,
            float straight, float strafe, float rotate, boolean slow, boolean quickR) {
        /** LIFT */
        lift.run(liftUp, liftDown, liftDirectUp, liftDirectDown);

        /** GRABBER */
        if (flipped) {
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
        doRotation(rot);
        doRotFix(fixRot);

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
    public void drive(
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
        doRotation(rot);
        doRotFix(fixRot);

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

    /** STOP METHODS */
    public void stop() {

        /** DRIVE */
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);

        /** LIFT */
        lift.stop();

        /** GRABBER */
        rot.setPower(0);

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

    /** ROTATION */
    private void doRotation (boolean b) {
        if (rotating) {
            if (Lift.update_encoders(rot)) { // Done rotating
                rotating = false;
                flipped = !flipped;
                if (lifted) { lift.ground(); lifted = false; }
                return;
            } else return;
        }
        if (rotated && !b) { rotated = false; }
        if (!b) return;
        if (!rotated) { // Start rotating
            if (lift.grounded()) { lifted = true; lift.setPosition(flip_position); }
            if (flipped) {
                rot.setTargetPosition(nflipped_pos);
                rot.setPower(flip_power);
            } else {
                rot.setTargetPosition(flipped_pos);
                rot.setPower(flip_power);
            }
            rotating = true;
        }
    }
    private void doRotFix (boolean b) {
        if (fixing && Lift.update_encoders(rot)) {
            rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (lifted) { lift.ground(); lifted = false; }
            fixing = false;
            return;
        }
        if (fixed && !b) { fixed = false; return; }
        if (!b) return;
        if (!fixed) {
            if (lift.grounded()) { lifted = true; lift.setPosition(flip_position); }
            rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rot.setTargetPosition(-flipped_pos);
            rot.setPower(-flip_power);
            fixing = true;
        }
    }

    /** AUTONOMOUS */
    public void activateVuforia () { visuals.vuforia.start(); }
    public String keyColumn () { return keyColumn.toString(); }
    public String checkColumn () { keyColumn = Column.look(visuals.vuforia); return keyColumn(); }

    public void wait(double seconds) {
        ElapsedTime time = new ElapsedTime();
        while (time.seconds() < seconds) ;
    }

    public void doParkAutonomous (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;
        doJewels();
        if (alliancePosition.justDoJewel() || !opmode.opModeIsActive()) return;
        doPark(opmode);
        closeVisuals();
    }

    public void doJewels () {
        br.close();
        bl.close();
        tr.open();
        tl.open();
        lowerArm();
        calibrateGyro();
        Color jewelC = Color.readCS(jewelSensor);
        if (jewelC == Color.ERROR) { raiseArm(); return; }
        if (allianceColor.turnCCW(jewelC)) {
            turnPastAngle(ccwAngle, true, .5f);
        } else turnPastAngle(cwAngle, false, .5f);
        raiseArm();
    }

    public void doPark (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;
        turnPastAngle(alliancePosition.getParkingAngle(allianceColor), allianceColor.isBoxToLeft(), .5f);
        if (!opmode.opModeIsActive()) return;
        lift.lift();
        lift.waitForEncoders();
        driveDistance(alliancePosition.getParkingDistance());
        lift.groundground();
        lift.waitForEncoders();
    }

    public VuforiaHandler.PosRot driveOffBalance (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return null;

        br.close();
        bl.close();
        tr.open();
        tl.open();
        raiseArm();

        turnToAngle(0,theta()>180,.33f);

        //VuforiaHandler.PosRot pr = visuals.vuforia.getRelativePosition(); // check position before coming off
        keyColumn = Column.look(visuals.vuforia); // store key column before coming off

        lift.setPosition(flip_position);
        lift.waitForEncoders();

        driveDistance(36);
        //wait(1.0);

        VuforiaHandler.PosRot pr = visuals.vuforia.getRelPosWithAverage(3);
        //pr.position.z += 30; // About the error of the pose calculation
        pr.position.mmToInches();

        int starting_theta = theta();

//        opmode.telemetry.addData("z_pos", Math.abs(pr.position.z));
//        opmode.telemetry.update();
//        wait(5.0);
//        if (!opmode.opModeIsActive()) return null;

//        double z_pos; // Make sure we are around 54 inches from pictograph
//        if ((z_pos = Math.abs(pr.position.z)) < 54 && z_pos > 30) {
//            opmode.telemetry.addData("Status", "Should be driving away.");
//            opmode.telemetry.update();
//            driveDistance(54.0 - z_pos, .2f);
//        }

        if (allianceColor == Color.BLUE) { // Rotate CCW if blue
            int goal_theta = 90 - (int)(theta() + pr.rotation.y);
            if (goal_theta < 0) goal_theta += 360;
            goal_theta = 92;
            opmode.telemetry.addData("Goal", goal_theta);
            opmode.telemetry.update();
            turnToAngle(goal_theta, true, .5f);
            turnToAngle(goal_theta, false, .2f);
        }

        if (allianceColor == Color.RED) { // Rotate CW if red
            int goal_theta = (int)(theta() + pr.rotation.y) - 90;
            if (goal_theta < 0) goal_theta += 360;
            goal_theta = 268;
            opmode.telemetry.addData("Goal", goal_theta);
            opmode.telemetry.update();
            turnToAngle(goal_theta, false, .5f);
            turnToAngle(goal_theta, true, .2f);
        }

        int end_theta = theta();
        int wall_theta = Math.round(pr.rotation.y);

        pr.position = Vector3.sum(pr.position, calcPhonePositionDelta(end_theta - starting_theta));
        if (allianceColor == Color.BLUE) pr.rotation.x = ((90+wall_theta+starting_theta)-end_theta);
        //wait(3.0);

        lastKnownPos = pr;
        //lift.groundground();
        return pr;
        // drive off keeping an eye on the gyro and the vumark
        // if the vumark goes out of sight, note that it does and use the encoders to guess distance
        //      keep checking if it appears again, if not just base movements off of encoders
        //      if it does, get pos data and position self about four and a half feet from it

    }
    private static Vector3 calcPhonePositionDelta(int delta_theta) {
        return new Vector3(
                5.23f*((float)Math.cos(Math.toRadians(39.4 + delta_theta)) - .7727f),
                0,
                5.23f*(.6347f - (float)Math.sin(Math.toRadians(39.4 + delta_theta)))
        );
    }

    public void driveToCryptobox (LinearOpMode opmode) {
        if (!opmode.opModeIsActive() || lastKnownPos == null) return;

        double d = 0;
        if (allianceColor == Color.RED) d = 17.5 - lastKnownPos.position.x;
        if (allianceColor == Color.BLUE) d = 32.5 + lastKnownPos.position.x;

//        opmode.telemetry.addData("x", lastKnownPos.position.x);
//        opmode.telemetry.update();
//        wait(3.0);

        Vector3 straightDrift = driveDistanceDrift(d);

        d = keyColumn.inchesToColumnFromVumark(alliancePosition.toID() + allianceColor.toID());
        boolean right = d > 0;
        d = Math.abs(lastKnownPos.position.z) - Math.abs(d) + 3;
        if (allianceColor == Color.BLUE && alliancePosition == Position.CORNER && keyColumn == Column.LEFT) d += 6;
        if (allianceColor == Color.BLUE && alliancePosition == Position.CORNER && keyColumn == Column.RIGHT) d -= 3;

        Vector3 strafeDrift = strafeDistanceDrift(d, right);

        Vector3 s2drift = strafeDistanceDrift(straightDrift.y + 1, allianceColor == Color.BLUE);

//        straightDrift.teleout(opmode, "straight");
//        strafeDrift.teleout(opmode, "strafe");
//        s2drift.teleout(opmode, "s2");
//        opmode.telemetry.addData("Off by", lastKnownPos.rotation.x / 2);
//        opmode.telemetry.update();
//        wait(10.0);

        //lift.groundground();
        // using pos data, calculate turn angle and distance to drive to line up with box wall
        // turn using the gyro such that the robot is facing the cryptobox
        // drive the distance using the motor encoders while making sure we stay straight
        // strafe the remaining distance based off of pos data so that the glyph is in front of the key column
        // IF NEEDED: use the phones front camera to align further to the cryptobox
    }

    public void placeGlyph (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        lift.ground();
        lift.waitForEncoders();

        driveDistance(12, .3f);

        bl.open();
        br.open();

        driveDistance(-4, .3f);

        lift.groundground();
        lift.waitForEncoders();

        // drive into the cryptobox, perhaps using an ultrasonic sensor to monitor the distance from the wall
        // release glyph from grabber
        // back away from cryptobox
    }

    public void driveDistance (double inches) {
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
    public void driveDistance (double inches, float power) {
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
    public Vector3 driveDistanceDrift (double inches) {
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
//        opmode.telemetry.addData("goal", e_goal);
//        opmode.telemetry.update();
//        wait(2.0);
        setDrivePower(.5);
        while (!((cur_e = driveCurPosition()) < e_goal + Lift.thres && cur_e > e_goal - Lift.thres)) {
            int de = cur_e - last_e;
            int dtheta = gyro.getHeading() - start_theta;
            drift.x += (Math.PI * (float)de * Math.cos(Math.toRadians(dtheta)))/280f;
            drift.y += (Math.PI * (float)de * Math.sin(Math.toRadians(dtheta)))/280f;
            last_e = cur_e;
            opmode.telemetry.addData("dx", drift.rx());
            opmode.telemetry.addData("dy", drift.ry());
            opmode.telemetry.update();
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

    public void strafeDistance (int count, boolean right) {
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
        while (!checkDriveEncoders()) ;
        setDrivePower(0);
    }
    public void strafeAccelDistance (int count, double pPerMS, boolean right) {
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
        while (!checkDriveEncoders()) {
            power = 1000.0 * pPerMS * time.seconds();
            if (power > .5) power = .5;
            setDrivePower(power);
            opmode.telemetry.addData("power", power);
            opmode.telemetry.addData("time", time.seconds());
            opmode.telemetry.update();
        }
        setDrivePower(0);
    }
    public Vector3 strafeDistanceDrift (double inches, boolean right) {
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

    private void turnPastAngle(int ang, boolean ccw, float speed) {
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
    public void turnToAngle(int ang, boolean ccw, float speed) {
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

    private int driveCurPosition () {
        return (rf.getCurrentPosition() + rb.getCurrentPosition() + lf.getCurrentPosition() + lb.getCurrentPosition()) / 4;
    }
    private int driveNewCurPosition () {
        return (Math.abs(rf.getCurrentPosition()) +
                Math.abs(rb.getCurrentPosition()) +
                Math.abs(lf.getCurrentPosition()) +
                Math.abs(lb.getCurrentPosition())) / 4;
    }
    private boolean checkDriveEncoders () {
        if (rf.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return false;
        int
                r_rf = Math.abs(rf.getTargetPosition() - rf.getCurrentPosition()),
                r_rb = Math.abs(rb.getTargetPosition() - rb.getCurrentPosition()),
                r_lf = Math.abs(lf.getTargetPosition() - lf.getCurrentPosition()),
                r_lb = Math.abs(lb.getTargetPosition() - lb.getCurrentPosition());
        return ((r_rf+r_rb+r_lf+r_lb)/4) < Lift.thres;
    }

    private void setDrivePower (double power) {
        rf.setPower(power); rb.setPower(power); lf.setPower(power); lb.setPower(power);
    }

    public void tellDriverRelPos() { visuals.vuforia.tellDriverRelPos(); }

    @Deprecated public void driveToBox (FIELDPOS pos) {
        lift.lift();
        lift.waitForEncoders();
        driveDistance(pos.inchesToBox);
        lift.groundground();
        lift.waitForEncoders();
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
