package org.firstinspires.ftc.teamcode.mecanlift.controller;

// Created on 1/28/2018 at 11:29 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.mecanlift

import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;

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

import java.util.Vector;

import static org.firstinspires.ftc.teamcode.visuals.Vector3.round;

public class Mecanlift {

    /** ENUMS */
    /** COLOR ENUM FOR JEWELS */
    public enum Color {
        RED, BLUE, ERROR, RED_PARK, BLUE_PARK;
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
            switch (removePark()) {
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
        public boolean parkAuto() { return this == RED_PARK || this == BLUE_PARK; }
        public Color removePark() {
            if (!parkAuto()) return this;
            switch (this) {
                case RED_PARK: return RED;
                case BLUE_PARK: return BLUE;
                default: return ERROR;
            }
        }
    }

    /** POSITION ENUM FOR PARKING */
    public enum Position {
        SIDE, CORNER, JEWEL_ONLY, PARK, ERROR;
        public boolean justDoJewel() { return this == JEWEL_ONLY; }
        public int toID () {
            switch (this) {
                case SIDE: return side_id;
                case CORNER: return corner_id;
                default: return 0;
            }
        }
        public int getParkingAngle (Color allianceColor) {
            switch (this.toID() + allianceColor.removePark().toID()) {
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
            switch (alliancePositionID) { // < 0 means left
                case corner_id + red_id:    return ret+column_adjustment-phone_glyph_offset;
                case side_id + red_id:      /*ret -= mark_offset_inches; return -(ret+column_adjustment-phone_glyph_offset)*/ return -column_adjustment;
                case corner_id + blue_id:   return column_adjustment-ret-phone_glyph_offset;
                case side_id + blue_id:     /*ret += mark_offset_inches; return ret-column_adjustment+phone_glyph_offset;*/ return -column_adjustment;
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
            blo = 0.157,            // Bottom left open
            blc = 0.392,            // Bottom left close
            bro = 0.863,            // Bottom right open
            brc = 0.647,            // Bottom right close
            tlo = 0.863,            // Top left open
            tlc = 0.627,            // Top left close
            tro = 0.196,            // Top right open
            trc = 0.412,            // Top right close

            jewelArm_down = .2,    // The jewel servo position for lowering the arm
            jewelArm_up = 1,        // The jewel servo position for raising the arm

            sp_distance = 36.0,     // Distance in inches to drive in order to park from side position
            cp_distance = 32.8;     // Distance in inches to drive in order to park from corner position

    private static final int
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
            blue_id = 7,            // Number ID for blue alliance color

            BUTTON_ID = 3141592;

    private static final Point
            red_center = new Point(621, 1095),
            blue_center = new Point(378, 1107);

    private static final float      // Based off notes from 020618
            box_start_inches = 36f,
            mark_offset_inches = 10.5f,
            column_adjustment_inches = 7.63f,
            phone_glyph_offset = 3.95f,
            z_off_balance = 54f;

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
    private Rotater rot;

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
    public VuforiaHandler.PosRot lastKnownPos;

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

            visuals.vertx = alignment_x;
            visuals.hory = alignment_y;
            visuals.red.center = red_center;
            visuals.red.radius = circle_radius;
            visuals.blue.center = blue_center;
            visuals.blue.radius = circle_radius;
            VisualsHandler.phoneLightOn();
            visuals.showAlignmentCircles();
            activateVuforia();
            time = new ElapsedTime();

            visuals.getPreview().setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) { gettingem = true; visuals.getPreview().setOnClickListener(null); }
            });

            /** SENSORS */
            jewelSensor = (ModernRoboticsI2cColorSensor)opmode.hardwareMap.colorSensor.get(colorN);
            csLightOn();

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
    public int specialTheta() {
        int theta = theta();
        if (theta > 180) theta -= 360;
        return theta;
    }

    public void csLightOff() { opmode.hardwareMap.colorSensor.get(colorN).enableLed(false); }
    public void csLightOn() { opmode.hardwareMap.colorSensor.get(colorN).enableLed(true); }

    @Deprecated public Mecanlift (OpMode om, /*boolean auto,*/ Color allianceColor) { init(om, true, allianceColor, Position.ERROR); }

    /** INIT LOOP METHODS */
    public void showAligning () { if (((int)(time.seconds()%2) == 1)) visuals.showAlignmentCircles(); }
    private boolean gotem = false, gettingem = false;
    public void doFullInitLoop () {
        showAligning();
        tele("Seeing?", checkColumn());
        if (gettingem) {
            tele("Gotem", gotem);
            VuforiaHandler.PosRot pr = visuals.vuforia.getRelativePosition();
            if (pr.position.z != 0 && lastKnownPos == null) {
                lastKnownPos = pr;
                gotem = true;
            } else if (pr.position.z != 0) {
                lastKnownPos.doAverage(pr);
                lastKnownPos.position.teleout(opmode, "Pos");
                lastKnownPos.rotation.teleout(opmode, "Rot");
            } else if (gotem) {
                lastKnownPos = null;
                gotem = false;
            }
        }
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
        out = new ContinuousServo(opmode.hardwareMap.servo.get("out"));
        grab = new ToggleServo(opmode.hardwareMap.servo.get("grab"), grab_no_grab, grab_grab);
        up = new ToggleServo(opmode.hardwareMap.servo.get("up"), up_down, up_up);
        //out.stop();
    }
    private void startRelicArm() {
        out.stop();
        up.open();
        grab.open();
    }
    private void doRelicArm (float outPower, boolean togGrab, boolean togUp) {
        out.setPower(outPower);
        grab.tob(togGrab);
        up.tob(togUp);
    }
    private void stopRelicArm () { out.stop(); }

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
    public void activateVuforia () { visuals.vuforia.start(); }
    public String keyColumn () { return keyColumn.toString(); }
    private String checkColumn () { keyColumn = Column.look(visuals.vuforia); return keyColumn(); }

    @Deprecated private void wait(double seconds) {
        ElapsedTime time = new ElapsedTime();
        while (time.seconds() < seconds) ;
    }
    private void wait(LinearOpMode opmode, double seconds) {
        ElapsedTime time = new ElapsedTime();
        while (time.seconds() < seconds) if (!opmode.opModeIsActive()) return;
    }

    private void doParkAutonomous (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        telewithup("Status", "Doing jewels...");
        doJewels(opmode);

        if (alliancePosition.justDoJewel()) return;

        telewithup("Status", "Parking...");
        doPark(opmode);

        VisualsHandler.phoneLightOff();

        while (opmode.opModeIsActive()) {
            tele("Status", "Done.");
            lastKnownPos.position.teleout(opmode, "Pos");
            lastKnownPos.rotation.teleout(opmode, "Rot");
            tele("Θ", specialTheta());
            telewithup("Column", keyColumn());
        }

        closeVisuals();
    }

    public void doFullAutonomous (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;
        if (allianceColor.parkAuto()) { doParkAutonomous(opmode); return; }

        telewithup("Status", "Doing jewels...");
        doJewels(opmode);

        if (opmode.opModeIsActive() && alliancePosition == Position.CORNER) {
            telewithup("Status", "Driving off balancing plate...");
            driveOffBalance(opmode);

            telewithup("Status", "Driving to cryptobox...");
            driveToCryptobox(opmode);

            telewithup("Status", "Placing glyph and parking...");
            placeGlyph(opmode);
        } else if (opmode.opModeIsActive() && alliancePosition == Position.SIDE) {
            telewithup("Status", "Driving to cryptobox...");
            doSideToCryptobox(opmode);

            telewithup("Status", "Placing glyph and parking...");
            placeGlyph(opmode);
        }

        VisualsHandler.phoneLightOff();

        while (opmode.opModeIsActive()) {
            tele("Status", "Done.");
            lastKnownPos.position.teleout(opmode, "Pos");
            lastKnownPos.rotation.teleout(opmode, "Rot");
            tele("Θ", specialTheta());
            telewithup("Column", keyColumn());
        }

        closeVisuals();
    }

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

        if (allianceColor.turnCCW(jewelC)) {
            telewithup("Status", "Turning left...");
            turnPastAngle(ccwAngle, true, .5f);
        } else {
            telewithup("Status", "Turning right...");
            turnPastAngle(cwAngle, false, .5f);
        }
        telewithup("Status", "Raising arm...");
        raiseArm();
        telewithup("Status", "Completed doing jewel.");
    }
    private void doJewels (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

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

        if (!opmode.opModeIsActive()) return;
        if (allianceColor.turnCCW(jewelC)) {
            telewithup("Status", "Turning left...");
            turnPastAngle(ccwAngle, true, .5f);
        } else {
            telewithup("Status", "Turning right...");
            turnPastAngle(cwAngle, false, .5f);
        }
        telewithup("Status", "Raising arm...");
        raiseArm();
        telewithup("Status", "Completed doing jewel.");
    }

    private void doPark (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;
        turnPastAngle(alliancePosition.getParkingAngle(allianceColor), allianceColor.isBoxToLeft(), .5f);
        if (!opmode.opModeIsActive()) return;
        lift.setPosition(flip_position);
        lift.waitForEncoders();
        driveDistance(alliancePosition.getParkingDistance());
        lift.groundground();
        lift.waitForEncoders();
    }

    private void driveOffBalance (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        telewithup("Status", "Opening/closing/raising everything...");
        br.close();
        bl.close();
        tr.open();
        tl.open();
        csLightOff();
        raiseArm(); // Open/close/raise everything before driving off

        telewithup("Status", "Getting straight and raising lift...");
        lift.setPosition(flip_position); // Start raising the grabber before coming off so the glyph doesn't hit the ground
        getStraight(); // Straighten up before driving off

        telewithup("Status", "Saving key column...");
        keyColumn = Column.look(visuals.vuforia); // store key column before coming off
        if (keyColumn == Column.ERROR) {
            telewithup("Status", "WARNING: COULD NOT SEE VUMARK!");
            wait(opmode, 3);
        }

        telewithup("Status", "Waiting on lift...");
        lift.waitForEncoders(); // Wait for the lift if its not completed

        telewithup("Status", "Driving arbitrarily off plate...");
        driveDistance(36); // Drive an arbitrary three feet off of plate

        telewithup("Status", "Getting position...");
        VuforiaHandler.PosRot pr = visuals.vuforia.getRelPosWithAverage(3); // Get position and average it for three seconds (for accuracy)
        pr.position.mmToInches(); // Convert it to inches (reports in mm)

        if (Math.abs(pr.position.z) < z_off_balance) {
            float d = z_off_balance - Math.abs(pr.position.z);
            telewithup("Status", "Fixing Z " + Vector3.round(d) + " inches...");
            if (d < 5) driveDistance(d, .3f);
        }

        telewithup("Status", "Aligning to vumark...");
        turnToSpecialAngle((int)lastKnownPos.rotation.y, .15f); // Align to pictograph to get good position

        telewithup("Status", "Getting position...");
        pr = visuals.vuforia.getRelPosWithAverage(3); // Get position and average it for three seconds (for accuracy)
        pr.position.mmToInches(); // Convert it to inches (reports in mm)

        int starting_theta = theta(); // Take note of the angle we start at before turning

        if (allianceColor == Color.BLUE) { // Rotate CCW if blue
            int goal_theta = specialTheta() + 90;
            telewithup("Status", "Turning left to " + goal_theta + " degrees...");
            turnToAngle(goal_theta, true, .33f);
            turnToAngle(goal_theta, false, .2f);
        }

        if (allianceColor == Color.RED) { // Rotate CW if red
            int goal_theta = specialTheta() + 270;
            telewithup("Status", "Turning right to " + goal_theta + " degrees...");
            turnToAngle(goal_theta, false, .33f);
            turnToAngle(goal_theta, true, .2f);
        }

        int end_theta = theta(); // Our gyro angle after rotating

        Vector3 phoneD = calcPhonePositionDelta(end_theta - starting_theta); // Calculate phone delta (based off math from Mr. Riehm)

        tele("Status", "Calculated phone delta with difference of " + (end_theta - starting_theta) + " degrees.");
        phoneD.teleout(opmode, "phone Δ");
        teleup();

        pr.position = Vector3.sum(pr.position, phoneD); // Add the change in the phones position to our position variable

        pr.rotation.x = starting_theta; pr.rotation.y = lastKnownPos.rotation.y; pr.rotation.z = end_theta; // Store these for later use

        lastKnownPos = pr; // Save the position from before into a variable for use later

    }
    private static Vector3 calcPhonePositionDelta(float delta_theta) {
        return new Vector3(
                5.23f*((float)Math.cos(Math.toRadians(39.4 + delta_theta)) - .7727f),
                0,
                5.23f*(.6347f - (float)Math.sin(Math.toRadians(39.4 + delta_theta)))
        );
    }
    private static Vector3 newCalcPhonePositionDelta(int theta0, int theta1, int theta2) {
        Vector3 a = f(theta1-theta0), b = f(theta2-theta0);
        return new Vector3((a.x*-1)+b.x, (a.y*-1)+b.y, 0);
    }
    private static Vector3 f(int delta_theta) { // Mr. Riehm's algorithm
        return new Vector3(
                5.23f*((float)Math.cos(Math.toRadians(39.4 + delta_theta)) - (float)Math.cos(Math.toRadians(39.4))),
                5.23f*((float)Math.sin(Math.toRadians(39.4)) - (float)Math.sin(Math.toRadians(39.4 + delta_theta))),
                0
        );
    }

    private void driveToCryptobox (LinearOpMode opmode) {
        if (!opmode.opModeIsActive() || lastKnownPos == null) return;
        VisualsHandler.phoneLightOff();

        // Calculate the distance to drive straight (based on notes)
        double d = 0;
        if (allianceColor == Color.RED) d = 17.5 - lastKnownPos.position.x;
        if (allianceColor == Color.BLUE) d = 37.5 + lastKnownPos.position.x;

        // Drive the distance, watching our angle and calculating drift
        telewithup("Status", "Driving straight " + Vector3.round(((float)d)) + " inches...");
        Vector3 straightDrift = driveDistanceDrift(d);

        // Update our position
        tele("Status", "Updating position...");
        straightDrift.teleout(opmode, "Drift");
        teleup();
        if (allianceColor == Color.BLUE) {
//            int start = specialTheta();
//            turnBackToSpecialAngle((int)lastKnownPos.rotation.y + 90, .15f);
//            int end = specialTheta();
//            lastKnownPos.position = Vector3.sum(lastKnownPos.position, calcPhonePositionDelta(end-start));
            lastKnownPos.position.x -= d + straightDrift.x;
            lastKnownPos.position.z += straightDrift.y;
        }
        if (allianceColor == Color.RED) {
//            int start = specialTheta();
//            turnBackToSpecialAngle((int)lastKnownPos.rotation.y + 270, .15f);
//            int end = specialTheta();
//            lastKnownPos.position = Vector3.sum(lastKnownPos.position, calcPhonePositionDelta(end-start));
            lastKnownPos.position.x += d + straightDrift.x;
            lastKnownPos.position.z -= straightDrift.y;
        }

        // Calculate the distance to strafe based on math from notes
        telewithup("Status", "Calculating strafing distance...");
        d = keyColumn.inchesToColumnFromVumark(alliancePosition.toID() + allianceColor.toID());
        boolean right = d > 0;
        d = Math.abs(lastKnownPos.position.z) - Math.abs(d);

        // Add corrections from testing
        if (allianceColor == Color.BLUE && alliancePosition == Position.CORNER) {
            d += 3;
            if (keyColumn == Column.LEFT) d += 6;
        } else if (allianceColor == Color.RED && alliancePosition == Position.CORNER) {
            d -= 1;
            if (keyColumn == Column.LEFT) d -= 8;
        }

        // If we don't have pos data, let the drivers know and strafe a foot
        if (lastKnownPos.position.z == 0) {
            telewithup("WARNING", "There was no Z value. Guessing the distance to drive...");
            d = 12;
        }

        // Strafe the distance
        telewithup("Status", "Strafing " + Vector3.round(((float)d)) + " inches...");
        Vector3 strafeDrift = strafeDistanceDrift(d, right);

        // Update our position
        lastKnownPos.position.z += d + strafeDrift.x;
        lastKnownPos.position.x += strafeDrift.y;

        telewithup("Status", "Done driving to cryptobox...");
    }

    private void placeGlyph (LinearOpMode opmode) {
        if (!opmode.opModeIsActive()) return;

        lift.ground();
        lift.waitForEncoders();

        driveDistance(12, .3f);

        bl.open();
        br.open();

        driveDistance(-4, .3f);

        lift.groundground();
        lift.waitForEncoders();
    }

    private void doSideToCryptobox (LinearOpMode opmode) {
        if (!opmode.opModeIsActive() || lastKnownPos == null) return;
        VisualsHandler.phoneLightOff();

//        int start_theta = specialTheta();

        lift.setPosition(flip_position);

//        // Turn towards cryptobox
//        if (allianceColor == Color.BLUE) {
//            turnToAngle((int)lastKnownPos.rotation.y + 90, true, .3f);
//        } else if (allianceColor == Color.RED) {
//            turnToAngle((int)lastKnownPos.rotation.y + 270, false, .3f);
//        }
        getPerpendicular();

        lift.waitForEncoders();

        // Drive three feet
        telewithup("Status", "Driving three feet...");
        Vector3 strafeDrift = strafeDistanceDrift(36, allianceColor != Color.BLUE);

        lastKnownPos.position.mmToInches();

//        int end_theta = specialTheta();
//
//        Vector3 phoneD = newCalcPhonePositionDelta((int)lastKnownPos.rotation.y, start_theta, end_theta);

        telewithup("Status", "Updating position...");
        lastKnownPos.position.z += strafeDrift.y;
        if (allianceColor == Color.BLUE) {
            lastKnownPos.position.x -= 36 + strafeDrift.x;
        } else if (allianceColor == Color.RED) {
            lastKnownPos.position.x += 36 + strafeDrift.x;
        }

//        telewithup("Status", "Aligning...");
//        if (allianceColor == Color.BLUE) {
//            turnToSpecialAngle((int)lastKnownPos.rotation.y + 90, .15f);
//        } else if (allianceColor == Color.RED) {
//            turnToSpecialAngle((int)lastKnownPos.rotation.y + 270, .15f);
//        }
//        getPerpendicular();

//        telewithup("Status", "Strafing four inches...");
//        Vector3 straightDrift = driveDistanceDrift(6);
//
//        lastKnownPos.position.z -= straightDrift.x;
//        lastKnownPos.position.x -= straightDrift.y;

        strafeDistanceDrift(2, allianceColor == Color.BLUE);

        getParallel();

        double d = keyColumn.inchesToColumnFromVumark(alliancePosition.toID() + allianceColor.toID());
        boolean right = d > 0;
        d = Math.abs(d) /*- Math.abs(lastKnownPos.position.x)*/;

        telewithup("Status", "Strafing " + Vector3.round(((float)d)) + " inches...");
        strafeDrift = strafeDistanceDrift(d, right);

        telewithup("Status", "Done driving to cryptobox from side...");
    }

    private void driveDistance (double inches) {
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
    private void driveDistance (double inches, float power) {
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
    private void turnToSpecialAngle(int ang, float power) {
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
            //setCWPower(power / 2f);
            //while ((theta = specialTheta()) > ang) telewithup("CW, theta", theta);
        } else if (theta > ang) { // CW
            lf.setPower(power);
            lb.setPower(power);
            while ((theta = specialTheta()) > ang) telewithup("CW, theta", theta);
            //setCCWPower(power / 2f);
            //while ((theta = specialTheta()) < ang) telewithup("CCW, theta", theta);
        }
        setDrivePower(0);
    }
    public void turnBackToSpecialAngle(int ang, float power) {
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
            //setCWPower(power / 2f);
            //while ((theta = specialTheta()) > ang) telewithup("CW, theta", theta);
        } else if (theta > ang) { // CW
            rf.setPower(-power);
            rb.setPower(-power);
            while ((theta = specialTheta()) > ang) telewithup("CW, theta", theta);
            //setCCWPower(power / 2f);
            //while ((theta = specialTheta()) < ang) telewithup("CCW, theta", theta);
        }
        setDrivePower(0);
    }
    private void getStraight() {
        int theta = specialTheta();
        if (theta > 0) { // Turn CW
            setCWPower(.33);
            while ((theta = specialTheta()) > 0) {
                tele("Status", "Getting straight... Pass 1");
                telewithup("CW, theta", theta);
            }
            setCCWPower(.2);
            while ((theta = specialTheta()) < 0) {
                tele("Status", "Getting straight... Pass 2");
                telewithup("CCW, theta", theta);
            }
        } else if (theta < 0) { // Turn CCW
            setCCWPower(.33);
            while ((theta = specialTheta()) < 0) {
                tele("Status", "Getting straight... Pass 1");
                telewithup("CCW, theta", theta);
            }
            setCWPower(.2);
            while ((theta = specialTheta()) > 0) {
                tele("Status", "Getting straight... Pass 2");
                telewithup("CW, theta", theta);
            }
        }
        setDrivePower(0);
        telewithup("Status", "Straightened.");
    }
    private void getParallel() { // To vumark

        getPerpendicular();

//        int theta = theta();
//        float goal = lastKnownPos.rotation.y + 180;
//        if (goal > 180) goal -= 360;
//        if (theta > goal) { // Turn CW
//            setCWPower(.33);
//            while ((theta = theta()) > goal) {
//                tele("Status", "Getting parallel... Pass 1");
//                tele("Goal", goal);
//                telewithup("CW, theta", theta);
//            }
//            setCCWPower(.2);
//            while ((theta = theta()) < goal) {
//                tele("Status", "Getting parallel... Pass 2");
//                tele("Goal", goal);
//                telewithup("CCW, theta", theta);
//            }
//        } else if (theta < goal) { // Turn CCW
//            setCCWPower(.33);
//            while ((theta = theta()) < goal) {
//                tele("Status", "Getting parallel... Pass 1");
//                tele("Goal", goal);
//                telewithup("CCW, theta", theta);
//            }
//            setCWPower(.2);
//            while ((theta = theta()) > goal) {
//                tele("Status", "Getting parallel... Pass 2");
//                tele("Goal", goal);
//                telewithup("CW, theta", theta);
//            }
//        }

        int theta = specialTheta(), goal = theta + 180;
        setCCWPower(.3);
        while (theta < goal) {
            tele("Status", "Getting parallel... Pass 1");
            tele("Goal", goal);
            telewithup("CCW, theta", theta);
            if ((theta = theta()) > 270) theta = specialTheta();
        }
        setCWPower(.2);
        while ((theta = theta()) > goal) {
            tele("Status", "Getting parallel... Pass 2");
            tele("Goal", goal);
            telewithup("CW, theta", theta);
        }
//        if (theta > goal) { // Turn CW
//            setCWPower(.33);
//            while ((theta = theta()) > goal) {
//                tele("Status", "Getting parallel... Pass 1");
//                tele("Goal", goal);
//                telewithup("CW, theta", theta);
//            }
//            setCCWPower(.2);
//            while ((theta = theta()) < goal) {
//                tele("Status", "Getting parallel... Pass 2");
//                tele("Goal", goal);
//                telewithup("CCW, theta", theta);
//            }
//        } else if (theta < goal) { // Turn CCW
//            setCCWPower(.33);
//            while ((theta = theta()) < goal) {
//                tele("Status", "Getting parallel... Pass 1");
//                tele("Goal", goal);
//                telewithup("CCW, theta", theta);
//            }
//            setCWPower(.2);
//            while ((theta = theta()) > goal) {
//                tele("Status", "Getting parallel... Pass 2");
//                tele("Goal", goal);
//                telewithup("CW, theta", theta);
//            }
//        }
        setDrivePower(0);
        telewithup("Status", "Paralleled.");
    }
    private void getPerpendicular() { // To vumark
        int theta = specialTheta();
        float goal = lastKnownPos.rotation.y;
        if (theta > goal) { // Turn CW
            setCWPower(.33);
            while ((theta = specialTheta()) > goal) {
                tele("Status", "Getting parallel... Pass 1");
                telewithup("CW, theta", theta);
            }
            setCCWPower(.2);
            while ((theta = specialTheta()) < goal) {
                tele("Status", "Getting parallel... Pass 2");
                telewithup("CCW, theta", theta);
            }
        } else if (theta < goal) { // Turn CCW
            setCCWPower(.33);
            while ((theta = specialTheta()) < goal) {
                tele("Status", "Getting parallel... Pass 1");
                telewithup("CCW, theta", theta);
            }
            setCWPower(.2);
            while ((theta = specialTheta()) > goal) {
                tele("Status", "Getting parallel... Pass 2");
                telewithup("CW, theta", theta);
            }
        }
        setDrivePower(0);
        telewithup("Status", "Paralleled.");
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
    private void setCCWPower (double power) {
        rf.setPower(power);
        rb.setPower(power);
        lf.setPower(-power);
        lb.setPower(-power);
    }
    private void setCWPower (double power) {
        rf.setPower(-power);
        rb.setPower(-power);
        lf.setPower(power);
        lb.setPower(power);
    }

    public void tellDriverRelPos() { visuals.vuforia.tellDriverRelPos(); }
    private void telewithup(String caption, Object data) {opmode.telemetry.addData(caption, data); opmode.telemetry.update(); }
    private void tele(String caption, Object data) { opmode.telemetry.addData(caption, data); }
    private void teleup () { opmode.telemetry.update(); }

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