package org.firstinspires.ftc.teamcode.robotHandlers;

import android.graphics.Path;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.AutonomousJorge;
import org.firstinspires.ftc.teamcode.robots.Jorge;

/**
 * Created by Chandler on 4/12/2017.
 */

public class JorgeAutonomousFunctions {

    private static final int
            LINE_THRESHOLD = 6275,
            ENCODER_RANGE = 10;

    private static final float
            IN_DISTANCE_PER_1680 = 19.125f;

    private static int direction = 1;

    public enum LINE_VALUES {
        MIDDLE_SENSOR (
                5900, // Line minimum for sensor 1
                5000, // Line edge minimum for sensor 1
                5900, // Line edge maximum for sensor 1
                5000  // Grey maximum for sensor 1
        ),
        BACK_SENSOR (
                5500, // Line minimum for sensor 1
                4500, // Line edge minimum for sensor 1
                5500, // Line edge maximum for sensor 1
                4500  // Grey maximum for sensor 1
        );

        public final int
                LINE,
                EDGE_MIN,
                EDGE_MAX,
                GREY;

        LINE_VALUES (int line, int edgemin, int edgemax, int grey) {
            LINE = line;
            EDGE_MIN = edgemin;
            EDGE_MAX = edgemax;
            GREY = grey;
        }
    }

    public enum LINE_POSITION { LINE, EDGE, GREY }

    @Deprecated public static void GO_TO_WHITE_LINE (AutonomousJorge jorge, AutonomousJorge.COLOR_SENSOR sensor) {
        jorge.drive.setAllPowers(.08);
        while (jorge.colorSensors.colorTemp(sensor.port) < LINE_THRESHOLD) {
            if (!jorge.opMode.opModeIsActive()) break;
            jorge.opMode.telemetry.addData("Status", "Looking for line...");
            jorge.opMode.telemetry.update();
        }
        jorge.drive.stopAll();
    }
    public static void GO_TO_WHITE_LINE (AutonomousJorge jorge) {
        if (!jorge.opMode.opModeIsActive()) return;

        // Verify not on line already
        if ( isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) return;


        // STEP 1

        // Telemetry so I know what's going on
        updateStatus(jorge, "Doing step 1 -- Go forward.");

        jorge.drive.setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);

        jorge.drive.setAllPowers( .1 ); // Set all motors powers in order to move straight

        // Go forward until the middle sensor is on the line
        while ( !isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ) { if (!jorge.opMode.opModeIsActive()) return; }
        jorge.drive.stopAll();

        if ( !isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ) GO_TO_WHITE_LINE(jorge);


        // STEP 2

        // Telemetry so I know what's going on
        updateStatus(jorge, "Doing step 2 -- Go backwards.");

        jorge.drive.setAllPowers( -.1 ); // Set all motors powers in order to move straight

        jorge.opMode.sleep(250);

        // Go backwards until the middle sensor isn't on the line
        while ( isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ) { if (!jorge.opMode.opModeIsActive()) return; }
        jorge.drive.stopAll();

    }

    @Deprecated public static void STRAIGHTEN_ON_WHITE_LINE (AutonomousJorge jorge, AutonomousJorge.COLOR_SENSOR primarySensor, AutonomousJorge.COLOR_SENSOR auxSensor) { if (!jorge.opMode.opModeIsActive() || primarySensor == auxSensor) return; jorge.drive.stopAll(); }
    public static void STRAIGHTEN_ON_WHITE_LINE (AutonomousJorge jorge) {
        if (!jorge.opMode.opModeIsActive()) return;

        // Verify not on line already
        if ( isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ||
                isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) GO_TO_WHITE_LINE(jorge);


        // STEP 3

        // Telemetry so I know what's going on
        updateStatus(jorge, "Doing step 3 -- Perfect rotation.");

        jorge.drive.setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);

        jorge.drive.setSidePowers( // Set side powers
                new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.LEFT, StandardRobotDrive.SIDE.RIGHT},
                                new double[]{                        -.125,                          .125});

        boolean middleSensorCrossedLine = false; // Variable to check whether the middle sensor crosses the line while turning

        while ( !isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) { // Rotate until the back sensor is on the line
            if (!jorge.opMode.opModeIsActive()) return;
            if ( isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ) middleSensorCrossedLine = true;
        }
        jorge.drive.stopAll();


        // STEP 4

        // Telemetry so I know what's going on
        updateStatus(jorge, "Doing step 4 -- Front rotation.");

        int rotationDirection = middleSensorCrossedLine ? -1 : 1;

        direction = rotationDirection;

        jorge.drive.setPowers( new String[]{ "lf", "rf" }, new double[]{ .2*(rotationDirection), -.2*(rotationDirection) } ); // Set front motor powers

        // Rotate until the middle sensor is on the line
        while ( !isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ) { if (!jorge.opMode.opModeIsActive()) return; }
        jorge.drive.stopAll();

        /*if (rotationDirection == -1) {

            jorge.drive.setPowers( new String[]{ "rf", "rb" }, .2 );

        }*/
    }

    public static void PRESS_BEACON (AutonomousJorge jorge, int noTimes) {

        if (!jorge.opMode.opModeIsActive() || noTimes < 1) return;

        final double
                BACK_UP_POWER = -.1,
                PRESS_POWER = .1,
                ROTATE_POWER = .175;
        final long
                PRESS_TIME = 1500,
                BACK_UP_TIME = 400,
                INTERVAL_TIME = 100;


        // STEP 5
        jorge.opMode.telemetry.addData("Status", "Doing step 5 -- Backup for middle sensor");
        jorge.opMode.telemetry.update();

        jorge.drive.setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);

        if (direction > 0) {
            jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{ (-ROTATE_POWER)-.075, 0, 0, (-ROTATE_POWER)-.075 } );
        } else {
            jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{0, (-ROTATE_POWER)-.075, (-ROTATE_POWER)-.075, 0} );
        }
        //jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{0, (-ROTATE_POWER)-.075, (-ROTATE_POWER)-.075, 0} );
        while ( !isSensorOnMiddle( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ) {
            if ( !jorge.opMode.opModeIsActive() ) return;
        }


        // STEP 6
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime runtime2 = new ElapsedTime();
        jorge.opMode.telemetry.addData("Status", "Doing step 6 -- Pressing/rotating");
        jorge.opMode.telemetry.update();

        jorge.servos.setPosition( Jorge.BEACON_PRESSER, Jorge.BEACON_DOWN );

        if (direction > 0) {
            jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{0, 0, ROTATE_POWER, ROTATE_POWER} );
        } else {
            jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{ROTATE_POWER, ROTATE_POWER, 0, 0} );
        }

        //jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{ROTATE_POWER, ROTATE_POWER, 0, 0} );
        boolean forwarding = false;
        runtime.reset();
        while ( runtime.milliseconds() < PRESS_TIME ) {
            if ( !jorge.opMode.opModeIsActive() ) return;
            if ( !forwarding && isSensorOnMiddle( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) {
                jorge.drive.setAllPowers(PRESS_POWER); forwarding = true;
            }
        }
        runtime2.reset();
        jorge.drive.stopAll();


        // STEP 7
        updateStatus( jorge, "Doing step 7 -- Backing up" );
        runtime.reset();
        while ( runtime.milliseconds() < INTERVAL_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
        jorge.drive.setAllPowers(-.2);
        runtime.reset();
        while ( runtime.milliseconds() < BACK_UP_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
        jorge.drive.stopAll();


        // STEP 8
        updateStatus( jorge, "Doing step 8 -- Pressing multiple times if needed" );
        noTimes--;
        for ( int i = 0; i < noTimes; i++ ) {

            while ( runtime2.milliseconds() < 6500 ) { if ( !jorge.opMode.opModeIsActive() ) return; }

            jorge.drive.setAllPowers(PRESS_POWER);
            runtime.reset();
            while ( runtime.milliseconds() < PRESS_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
            runtime2.reset();
            jorge.drive.stopAll();

            runtime.reset();
            while ( runtime.milliseconds() < INTERVAL_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }

            jorge.drive.setAllPowers(BACK_UP_POWER - .05);
            runtime.reset();
            while ( runtime.milliseconds() < BACK_UP_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
            jorge.drive.stopAll();
        }

        jorge.drive.stopAll();
        jorge.servos.setPosition( Jorge.BEACON_PRESSER, Jorge.BEACON_UP );

    }

    public static void FULL_PRESS_BEACON (AutonomousJorge jorge, AutonomousJorge.BEACON facingBeacon) throws InterruptedException, Error {

        /*PRESS_BEACON (jorge, 1);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        jorge.vuforia.checkOnBeacons(facingBeacon.index);
        if (jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_BLUE) {
            while (runtime.seconds() < 4) {jorge.opMode.telemetry.addData("Status", "Wrong color. Waiting to press..."); jorge.opMode.telemetry.update();}
            PRESS_BEACON(jorge, 1);
        }*/

        jorge.vuforia.checkOnBeacons(facingBeacon.index);
        //if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_BLUE ) throw new Error("Woah, that shouldn't happen. Config = RED, BLUE");
        int noTimes = 1;
        if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_RED ||
                jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_RED ) noTimes++;

        PRESS_BEACON( jorge, noTimes );

        jorge.vuforia.checkOnBeacons(facingBeacon.index);
        if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_BLUE ) {/* Press again. Code doesn't exist yet... */}

    }

    public static void TURN_TO_RELATIVE_ANGLE (AutonomousJorge jorge, int angle, Path.Direction dir) {

        jorge.drive.setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jorge.drive.setAllModes(DcMotor.RunMode.RUN_TO_POSITION);

        int encd = (int) Math.round(angle*((double)1000/47.66));

        if ( dir == Path.Direction.CW) {
            jorge.drive.setTargetPositions(
                    new String[]{ "lf", "lb",  "rf",  "rb" },
                    new int[]   { encd, encd, -encd, -encd },
                    new double[]{   .8,   .8,   -.8,   -.8 }
            );
        } else {
            jorge.drive.setTargetPositions(
                    new String[]{  "lf",  "lb", "rf", "rb" },
                    new int[]   { -encd, -encd, encd, encd },
                    new double[]{   -.8,   -.8,   .8,   .8 }
            );
        }

        while ( !compareTarget(jorge.drive.getAllPositions(), jorge.drive.getAllTargetPositions()) ) { if (!jorge.opMode.opModeIsActive()) return; }

        jorge.drive.stopAll();

        /*if (gyro.getHeading() != angle) {   Check doesn't work very well....
            rotateToAngle(angle, dir, gyro.getHeading(), encd);
        }*/

        jorge.drive.setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public static void TURN_TO_GYRO_ANGLE (AutonomousJorge jorge, int angle, ModernRoboticsI2cGyro gyro) {

        int relAngle = gyro.getHeading() - angle;

        //DebugLogger log = new DebugLogger();
        //log.log("" + relAngle);

        if (relAngle > 0) {
            if (relAngle > 180) {
                angle = 360-relAngle;
                TURN_TO_RELATIVE_ANGLE(jorge, angle, Path.Direction.CW);
            } else {
                TURN_TO_RELATIVE_ANGLE(jorge, relAngle, Path.Direction.CCW);
            }
        } else {
            if (relAngle < -180) {
                angle = 360+relAngle;
                TURN_TO_RELATIVE_ANGLE(jorge, angle, Path.Direction.CCW);
            } else {
                TURN_TO_RELATIVE_ANGLE(jorge, Math.abs(relAngle), Path.Direction.CW);
            }
        }


        //log.close_log();

    }

    public static void DRIVE_FORWARD_FEET ( AutonomousJorge jorge, float feet, double maxSpeed) {

        DRIVE_FORWARD_IN(jorge, feet*12, maxSpeed);

    }

    public static void DRIVE_FORWARD_IN ( AutonomousJorge jorge, float inches, double maxSpeed) {

        int distance = Math.round((inches / IN_DISTANCE_PER_1680) * 1680);
        boolean slowedDown = false;

        jorge.drive.setAllModes(DcMotor.RunMode.RUN_TO_POSITION);
        jorge.drive.setAllTargetPositions(distance, maxSpeed);

        while (!compareTarget(jorge.drive.getAllPositions(), jorge.drive.getAllTargetPositions())) {
            if (!jorge.opMode.opModeIsActive()) return;
            if (!slowedDown && jorge.drive.getPosition("rb") > Math.round(jorge.drive.getTargetPosition("rb")*(.75f))) {
                jorge.drive.setAllPowers(maxSpeed/2); slowedDown = true; }
        }
        jorge.drive.stopAll();

        jorge.drive.setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private static LINE_POSITION getLinePosition (AutonomousJorge jorge, LINE_VALUES sensorValues) {

        if ( sensorValues == LINE_VALUES.MIDDLE_SENSOR ) {
            int colorTemp = jorge.colorSensors.colorTemp( AutonomousJorge.COLOR_SENSOR.MIDDLE.port );
            if ( inRange( colorTemp, sensorValues.LINE, Integer.MAX_VALUE ) ) return LINE_POSITION.LINE;
            if ( inRange( colorTemp, sensorValues.EDGE_MIN, sensorValues.EDGE_MAX ) ) { return LINE_POSITION.EDGE; }
            else { return LINE_POSITION.GREY; }
        } else {
            int colorTemp = jorge.colorSensors.colorTemp( AutonomousJorge.COLOR_SENSOR.BACK.port );
            if ( inRange( colorTemp, sensorValues.LINE, Integer.MAX_VALUE ) ) return LINE_POSITION.LINE;
            if ( inRange( colorTemp, sensorValues.EDGE_MIN, sensorValues.EDGE_MAX ) ) { return LINE_POSITION.EDGE; }
            else { return LINE_POSITION.GREY; }
        }

    }

    private static boolean inRange (int comp, int min, int max) { return min < comp && comp < max; }

    private static void updateStatus ( AutonomousJorge jorge, String message ) { jorge.opMode.telemetry.addData("Status", message); jorge.opMode.telemetry.update(); }

    private static boolean isSensorOnLine ( AutonomousJorge jorge, AutonomousJorge.COLOR_SENSOR sensor ) {
        if (sensor == AutonomousJorge.COLOR_SENSOR.MIDDLE) {
            return getLinePosition(jorge, LINE_VALUES.MIDDLE_SENSOR) == LINE_POSITION.LINE;
        } else {
            return getLinePosition(jorge, LINE_VALUES.BACK_SENSOR) == LINE_POSITION.LINE;
        }
    }

    private static boolean isSensorOnMiddle ( AutonomousJorge jorge, AutonomousJorge.COLOR_SENSOR sensor ) {
        return squareRootDifference(jorge.colorSensors.getCRGB(sensor.port)) >= 60; }

    private static double squareRootDifference (int[] crgb) { return Math.pow((double)(crgb[0]*crgb[1]*crgb[2]*crgb[3]), .25); }

    private static boolean compareTarget ( int[] current, int[] target ) {
        if (current.length != target.length) return false;
        boolean Return = true;
        for ( int i = 0; i < current.length; i++ ) {
            if (current[i] > target[i]+ENCODER_RANGE || current[i] < target[i]-ENCODER_RANGE) {
                Return = false; break;
            }
        }
        return Return;
    }

}
