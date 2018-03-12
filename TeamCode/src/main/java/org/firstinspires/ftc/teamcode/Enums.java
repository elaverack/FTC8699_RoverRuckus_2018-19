package org.firstinspires.ftc.teamcode;

// Created on 3/4/2018 at 3:51 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.visuals.Vector3;
import org.firstinspires.ftc.teamcode.visuals.VuforiaHandler;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

public class Enums {

    private static final int // NOTE: corner refers to the position farthest away from the relic zones, side the closest.
            rc_park_turn_angle = 296,                   // Gyro angle to turn to park when the robot's stating position is the red corner
            rs_park_turn_angle = 290,                   // Gyro angle to turn to park when the robot's stating position is the red side
            bc_park_turn_angle = 64,                    // Gyro angle to turn to park when the robot's stating position is the blue corner
            bs_park_turn_angle = 70,                    // Gyro angle to turn to park when the robot's stating position is the blue side

            side_park_distance = 36,                    // Distance in inches to drive in order to park from side position
            center_park_distance = 32,                  // Distance in inches to drive in order to park from corner position

            corner_id = 1,                              // Number ID for corner position
            side_id = 4,                                // Number ID for side position
            red_id = 2,                                 // Number ID for red alliance color
            blue_id = 7;                                // Number ID for blue alliance color

    @Deprecated private static final float      // Based off notes from 020618
            box_start_inches = 36f,
            mark_offset_inches = 10.5f,
            column_adjustment_inches = 7.63f,
            phone_glyph_offset = 3.95f;

    @Deprecated private static final double
            crypto_center_to_most_left = 11.475,        //
            crypto_margin = 2.0;                        //

    /** COLOR ENUM FOR JEWELS */
    public enum Color {
        RED, BLUE, ERROR, RED_PARK, BLUE_PARK;

        public static Color readCS (ColorSensor s) {
            if (s.red() > s.blue()) return RED;
            if (s.blue() > s.red()) return BLUE;
            return ERROR;
        }

        public boolean turnCCWforJewels(Color jewelColor) {
            switch (this) {
                case RED: return jewelColor == BLUE;
                case BLUE: return jewelColor == RED;
                default: return false;
            }
        }
        public boolean turnCCWforJewels(JewelConfig jewelConfig) {
            switch (this) {
                case RED: return jewelConfig == JewelConfig.BLUE_RED;
                case BLUE: return jewelConfig == JewelConfig.RED_BLUE;
                default: return false;
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

        public int toID () {
            switch (this) {
                case RED: return red_id;
                case BLUE: return blue_id;
                default: return 0;
            }
        }
        public Scalar toScalar() {
            switch (this) {
                case BLUE: return new Scalar(0,0,255);
                case RED: return new Scalar(255,0,0);
                default: return new Scalar(255,255,255);
            }
        }
        public String toString() {
            switch (this.removePark()) {
                case RED: return "RED";
                case BLUE: return "BLUE";
                default: return "ERROR";
            }
        }
    }

    /** POSITION ENUM FOR PARKING */
    public enum Position {
        SIDE, CORNER, JEWEL_ONLY, ERROR;

        public boolean justDoJewel() { return this == JEWEL_ONLY; }

        public int getParkingAngle (Color allianceColor) {
            switch (this.toID() + allianceColor.removePark().toID()) {
                case corner_id + red_id: return rc_park_turn_angle;
                case side_id + red_id: return rs_park_turn_angle;
                case corner_id + blue_id: return bc_park_turn_angle;
                case side_id + blue_id: return bs_park_turn_angle;
                default: return 0;
            }
        }
        public double getParkingDistance () {
            switch (this) {
                case SIDE: return side_park_distance;
                case CORNER: return center_park_distance;
                default: return 0;
            }
        }

        public int getAwayFromCryptoAngle (Color allianceColor, float vumarkAng) {
            int vumarkChange = 0;
            switch (this.toID() + allianceColor.toID()) {
                case corner_id + red_id: vumarkChange = 90; break;
                case corner_id + blue_id: vumarkChange = -90; break;
            }
            return (int)vumarkAng + vumarkChange;
        }
        public int getFacingCryptoAngle (Color allianceColor, float vumarkAng) {
            int vumarkChange = 0;
            if (this == SIDE) vumarkChange = 180;
            switch (this.toID() + allianceColor.toID()) {
                case corner_id + red_id: vumarkChange = -90; break;
                case corner_id + blue_id: vumarkChange = 90; break;
            }
            return (int)vumarkAng + vumarkChange;
        }

        public int toID () {
            switch (this) {
                case SIDE: return side_id;
                case CORNER: return corner_id;
                default: return 0;
            }
        }

        @Deprecated public double distanceToAlignCryptobox (Color allianceColor, Vector3 position) { // GO RIGHT IF < 0
            if (this == JEWEL_ONLY || this == ERROR || position == null) return 0;
            double currentPosition = this == CORNER ? Math.abs(position.z) : 2;
            double distance = this == CORNER ? currentPosition - crypto_center_to_most_left - 36 : currentPosition + crypto_center_to_most_left;
            distance += crypto_margin;
            if (allianceColor == Color.BLUE) distance *= -1;
            return distance;
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

        public double doCryptoboxLogic (Color allianceColor, Position alliancePosition, double[] divs) {
            double ret = 0, ppi, phone_dis;
            
            switch (divs.length) {
                case 0: return 8;
                case 1:
                    double x = divs[0]; ret = 3;
                    if (allianceColor == Color.BLUE) { if (x > 630) ret = 8; else if (x > 480) ret = 6; }
                    else if (x < 90) ret = 8; else if (x < 240) ret = 6;
                    return ret;
                case 2: ppi = Math.abs(divs[1] - divs[0]) / 7.65; break;
                case 3: ppi = Math.abs(divs[2] - divs[0]) / 15.3; break;
                default: return ret;
            }

            if (allianceColor == Color.RED && alliancePosition == Position.CORNER) {
                phone_dis = (360 - divs[1]) / ppi;
                ret = phone_dis + 6.514;
                switch (this) {
                    case LEFT: return ret + 15.3;
                    case RIGHT: return ret;
                    case CENTER: return ret + 7.65;
                    default: return ret;
                }
            } else {
                phone_dis = (divs[0] - 360) / ppi;
                ret = phone_dis - 1.136;
                switch (this) {
                    case LEFT: return ret;
                    case RIGHT: return ret + 15.3;
                    case CENTER: return ret + 7.65;
                    default: return ret;
                }
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

        @Deprecated public double inchesToColumnFromVumark (int alliancePositionID) { // NOTE: return a v3 with the specific axis of distance changed
            if (this == ERROR) return 0;
            float ret = box_start_inches;
            float column_adjustment = 0;
            switch (this) {
                case LEFT:  column_adjustment += column_adjustment_inches;
                case RIGHT: column_adjustment -= column_adjustment_inches;
            }
            switch (alliancePositionID) { // < 0 means left
                case corner_id + red_id:    return ret+column_adjustment- phone_glyph_offset;
                case side_id + red_id:      /*ret -= mark_offset_inches; return -(ret+column_adjustment-phone_glyph_offset)*/ return -column_adjustment;
                case corner_id + blue_id:   return column_adjustment-ret- phone_glyph_offset;
                case side_id + blue_id:     /*ret += mark_offset_inches; return ret-column_adjustment+phone_glyph_offset;*/ return -column_adjustment;
                default:                    return 0;
            }
        }
    }

    /** JEWEL CONFIG ENUM */
    public enum JewelConfig {
        RED_BLUE("RED_BLUE"), BLUE_RED("BLUE_RED"), ERROR("ERROR");

        private final String toString;

        JewelConfig(String ts) { toString = ts; }

        public Color right() {
            switch (this) {
                case BLUE_RED: return Color.RED;
                case RED_BLUE: return Color.BLUE;
                default: return Color.ERROR;
            }
        }
        public Color left() {
            switch (this) {
                case BLUE_RED: return Color.BLUE;
                case RED_BLUE: return Color.RED;
                default: return Color.ERROR;
            }
        }

        public static JewelConfig intCircles (Color leftColor, Color rightColor) {
            if (leftColor == Color.RED && rightColor == Color.BLUE) return RED_BLUE;
            if (leftColor == Color.BLUE && rightColor == Color.RED) return BLUE_RED;
            return ERROR;
        }

        public String toString () { return toString; }

        @Deprecated public static JewelConfig intCOMs(Point red, Point blue) {
            if (red.x < blue.x) return RED_BLUE;
            return BLUE_RED;
        }
    }
}
