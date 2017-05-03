package org.firstinspires.ftc.teamcode.robots.jorge.autonomous;

import android.graphics.Path;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.archive.season2016_2017.testArchive.TurnToAngleTest;
import org.firstinspires.ftc.teamcode.robotHandlers.StandardRobotDrive;
import org.firstinspires.ftc.teamcode.robotHandlers.Vuforia2017Manager;
import org.firstinspires.ftc.teamcode.robots.jorge.AutonomousJorge;
import org.firstinspires.ftc.teamcode.robots.jorge.Jorge;

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

    public enum SIDE {
        BLUE, RED
    }

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
    public static void RED_STRAIGHTEN_ON_WHITE_LINE(AutonomousJorge jorge) {
        if (!jorge.opMode.opModeIsActive()) return;

        // Verify not on line already
        if ( isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ||
                isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) GO_TO_WHITE_LINE(jorge);


        // STEP 3

        // Telemetry so I know what's going on
        updateStatus(jorge, "Doing step 3 -- Perfect rotation.");

        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_UP );
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_R, AutonomousJorge.BEACON_PRESSER_R_UP );
        jorge.drive.setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);

        jorge.drive.setSidePowers( // Set side powers
                new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.LEFT, StandardRobotDrive.SIDE.RIGHT},
                                new double[]{                        -.125,                          .125});

        boolean middleSensorCrossedLine = false; // Variable to check whether the middle sensor crosses the line while turning

        while ( !isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) { // Rotate until the back sensor is on the line
            if (!jorge.opMode.opModeIsActive()) return;
            if ( isSensorOnMiddle( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ) middleSensorCrossedLine = true;
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
    public static void BLUE_STRAIGHTEN_ON_WHITE_LINE(AutonomousJorge jorge) {
        if (!jorge.opMode.opModeIsActive()) return;

        // Verify not on line already
        if ( isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ||
                isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) GO_TO_WHITE_LINE(jorge);


        // STEP 3

        // Telemetry so I know what's going on
        updateStatus(jorge, "Doing step 3 -- Perfect rotation.");

        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_UP );
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_R, AutonomousJorge.BEACON_PRESSER_R_UP );
        jorge.drive.setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);

        jorge.drive.setSidePowers( // Set side powers
                new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.LEFT, StandardRobotDrive.SIDE.RIGHT},
                new double[]{                        .125,                          -.125});

        boolean middleSensorCrossedLine = false; // Variable to check whether the middle sensor crosses the line while turning

        while ( !isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) { // Rotate until the back sensor is on the line
            if (!jorge.opMode.opModeIsActive()) return;
            if ( isSensorOnMiddle( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ) middleSensorCrossedLine = true;
        }
        jorge.drive.stopAll();


        // STEP 4

        // Telemetry so I know what's going on
        updateStatus(jorge, "Doing step 4 -- Front rotation.");

        int rotationDirection = middleSensorCrossedLine ? 1 : -1;

        direction = rotationDirection;

        jorge.opMode.telemetry.addData("Status", "" + direction);
        jorge.opMode.telemetry.update();

        jorge.drive.setPowers( new String[]{ "lf", "rf" }, new double[]{ .2*(rotationDirection), -.2*(rotationDirection) } ); // Set front motor powers

        // Rotate until the middle sensor is on the line
        while ( !isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE ) ) { if (!jorge.opMode.opModeIsActive()) return; }
        jorge.drive.stopAll();

        /*if (rotationDirection == -1) {

            jorge.drive.setPowers( new String[]{ "rf", "rb" }, .2 );

        }*/
    }

    @Deprecated public static void PRESS_BEACON (AutonomousJorge jorge, int noTimes) {

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

        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_DOWN );

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
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_UP );

    }
    public static void RED_PRESS_BEACON(AutonomousJorge jorge, TurnToAngleTest.Direction presser) {

        if (!jorge.opMode.opModeIsActive()) return;

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

        if ( presser == TurnToAngleTest.Direction.LEFT ) {
            jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_DOWN );
        } else {
            jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_R, AutonomousJorge.BEACON_PRESSER_R_DOWN );
        }

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
            if ( !forwarding && isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) {
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


//        updateStatus( jorge, "Doing step 8 -- Pressing multiple times if needed" );
//        noTimes--;
//        for ( int i = 0; i < noTimes; i++ ) {
//
//            while ( runtime2.milliseconds() < 6500 ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//
//            jorge.drive.setAllPowers(PRESS_POWER);
//            runtime.reset();
//            while ( runtime.milliseconds() < PRESS_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//            runtime2.reset();
//            jorge.drive.stopAll();
//
//            runtime.reset();
//            while ( runtime.milliseconds() < INTERVAL_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//
//            jorge.drive.setAllPowers(BACK_UP_POWER - .05);
//            runtime.reset();
//            while ( runtime.milliseconds() < BACK_UP_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//            jorge.drive.stopAll();
//        }

        jorge.drive.stopAll();
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_UP );
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_R, AutonomousJorge.BEACON_PRESSER_R_UP );

    }
    public static void BLUE_PRESS_BEACON(AutonomousJorge jorge, TurnToAngleTest.Direction presser) {

        if (!jorge.opMode.opModeIsActive()) return;

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
        //jorge.opMode.telemetry.update();

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
        //jorge.opMode.telemetry.update();

        if ( presser == TurnToAngleTest.Direction.LEFT ) {
            jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_DOWN );
        } else {
            jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_R, AutonomousJorge.BEACON_PRESSER_R_DOWN );
        }

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
            if ( !forwarding && isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) {
                jorge.drive.setAllPowers(PRESS_POWER); forwarding = true;
            }
        }
        runtime2.reset();
        jorge.drive.stopAll();


        // STEP 7
        //updateStatus( jorge, "Doing step 7 -- Backing up" );
        runtime.reset();
        while ( runtime.milliseconds() < INTERVAL_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
        jorge.drive.setAllPowers(-.2);
        runtime.reset();
        while ( runtime.milliseconds() < BACK_UP_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
        jorge.drive.stopAll();


//        updateStatus( jorge, "Doing step 8 -- Pressing multiple times if needed" );
//        noTimes--;
//        for ( int i = 0; i < noTimes; i++ ) {
//
//            while ( runtime2.milliseconds() < 6500 ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//
//            jorge.drive.setAllPowers(PRESS_POWER);
//            runtime.reset();
//            while ( runtime.milliseconds() < PRESS_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//            runtime2.reset();
//            jorge.drive.stopAll();
//
//            runtime.reset();
//            while ( runtime.milliseconds() < INTERVAL_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//
//            jorge.drive.setAllPowers(BACK_UP_POWER - .05);
//            runtime.reset();
//            while ( runtime.milliseconds() < BACK_UP_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//            jorge.drive.stopAll();
//        }

        jorge.drive.stopAll();
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_UP );
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_R, AutonomousJorge.BEACON_PRESSER_R_UP );

    }
    public static void RED_PRESS_BEACON(AutonomousJorge jorge, TurnToAngleTest.Direction presser, boolean doFinalCorrection) {

        if (!jorge.opMode.opModeIsActive()) return;

        final double
                BACK_UP_POWER = -.1,
                PRESS_POWER = .1,
                ROTATE_POWER = .175;
        final long
                PRESS_TIME = 3000,
                BACK_UP_TIME = 400,
                INTERVAL_TIME = 100;

        ElapsedTime runtime = new ElapsedTime();

        // STEP 5
        if (doFinalCorrection) {
            jorge.opMode.telemetry.addData("Status", "Doing step 5 -- Backup for middle sensor");
            jorge.opMode.telemetry.update();

            jorge.drive.setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);

            if (direction > 0) {
                jorge.drive.setPowers(new String[]{"rf", "rb", "lf", "lb"}, new double[]{(-ROTATE_POWER) - .075, 0, 0, (-ROTATE_POWER) - .075});
            } else {
                jorge.drive.setPowers(new String[]{"rf", "rb", "lf", "lb"}, new double[]{0, (-ROTATE_POWER) - .075, (-ROTATE_POWER) - .075, 0});
            }
            //jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{0, (-ROTATE_POWER)-.075, (-ROTATE_POWER)-.075, 0} );
            while (!isSensorOnMiddle(jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE)) {
                if (!jorge.opMode.opModeIsActive()) return;
            }
        } else {
            jorge.drive.setAllPowers(-.2);
            runtime.reset();
            while ( runtime.milliseconds() < BACK_UP_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
            jorge.drive.stopAll();
        }


        // STEP 6
        ElapsedTime runtime2 = new ElapsedTime();
        jorge.opMode.telemetry.addData("Status", "Doing step 6 -- Pressing/rotating");
        jorge.opMode.telemetry.update();

        if ( presser == TurnToAngleTest.Direction.LEFT ) {
            jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_DOWN );
        } else {
            jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_R, AutonomousJorge.BEACON_PRESSER_R_DOWN );
        }

        boolean forwarding = false;
        if (direction > 0 && doFinalCorrection) {
            jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{0, 0, ROTATE_POWER, ROTATE_POWER} );
        } else if (doFinalCorrection) {
            jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{ROTATE_POWER, ROTATE_POWER, 0, 0} );
        } else {
            jorge.drive.setAllPowers(PRESS_POWER); forwarding = true;
        }

        //jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{ROTATE_POWER, ROTATE_POWER, 0, 0} );

        runtime.reset();
        while ( runtime.milliseconds() < PRESS_TIME ) {
            if ( !jorge.opMode.opModeIsActive() ) return;
            if ( !forwarding && isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) {
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


//        updateStatus( jorge, "Doing step 8 -- Pressing multiple times if needed" );
//        noTimes--;
//        for ( int i = 0; i < noTimes; i++ ) {
//
//            while ( runtime2.milliseconds() < 6500 ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//
//            jorge.drive.setAllPowers(PRESS_POWER);
//            runtime.reset();
//            while ( runtime.milliseconds() < PRESS_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//            runtime2.reset();
//            jorge.drive.stopAll();
//
//            runtime.reset();
//            while ( runtime.milliseconds() < INTERVAL_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//
//            jorge.drive.setAllPowers(BACK_UP_POWER - .05);
//            runtime.reset();
//            while ( runtime.milliseconds() < BACK_UP_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//            jorge.drive.stopAll();
//        }

        jorge.drive.stopAll();
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_UP );
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_R, AutonomousJorge.BEACON_PRESSER_R_UP );

    }
    public static void BLUE_PRESS_BEACON(AutonomousJorge jorge, TurnToAngleTest.Direction presser, boolean doFinalCorrection) {

        if (!jorge.opMode.opModeIsActive()) return;

        final double
                BACK_UP_POWER = -.1,
                PRESS_POWER = .1,
                ROTATE_POWER = .175;
        final long
                PRESS_TIME = 3000,
                BACK_UP_TIME = 400,
                INTERVAL_TIME = 100;

        ElapsedTime runtime = new ElapsedTime();

        // STEP 5
        if (doFinalCorrection) {
            jorge.opMode.telemetry.addData("Status", "Doing step 5 -- Backup for middle sensor");
            //jorge.opMode.telemetry.update();

            jorge.drive.setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);

            if (direction > 0) {
                jorge.drive.setPowers(new String[]{"rf", "rb", "lf", "lb"}, new double[]{(-ROTATE_POWER) - .075, 0, 0, (-ROTATE_POWER) - .075});
            } else {
                jorge.drive.setPowers(new String[]{"rf", "rb", "lf", "lb"}, new double[]{0, (-ROTATE_POWER) - .075, (-ROTATE_POWER) - .075, 0});
            }
            //jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{0, (-ROTATE_POWER)-.075, (-ROTATE_POWER)-.075, 0} );
            while (!isSensorOnMiddle(jorge, AutonomousJorge.COLOR_SENSOR.MIDDLE)) {
                if (!jorge.opMode.opModeIsActive()) return;
            }
        } else {
            jorge.drive.setAllPowers(-.2);
            runtime.reset();
            while ( runtime.milliseconds() < BACK_UP_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
            jorge.drive.stopAll();
        }


        // STEP 6
        ElapsedTime runtime2 = new ElapsedTime();
        jorge.opMode.telemetry.addData("Status", "Doing step 6 -- Pressing/rotating");
        //jorge.opMode.telemetry.update();

        if ( presser == TurnToAngleTest.Direction.LEFT ) {
            jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_DOWN );
        } else {
            jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_R, AutonomousJorge.BEACON_PRESSER_R_DOWN );
        }

        boolean forwarding = false;
        if (direction > 0 && doFinalCorrection) {
            jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{0, 0, ROTATE_POWER, ROTATE_POWER} );
        } else if (doFinalCorrection) {
            jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{ROTATE_POWER, ROTATE_POWER, 0, 0} );
        } else {
            jorge.drive.setAllPowers(PRESS_POWER);
            forwarding = true;
        }

        //jorge.drive.setPowers( new String[]{"rf", "rb", "lf", "lb"}, new double[]{ROTATE_POWER, ROTATE_POWER, 0, 0} );

        runtime.reset();
        while ( runtime.milliseconds() < PRESS_TIME ) {
            if ( !jorge.opMode.opModeIsActive() ) return;
            if ( !forwarding && isSensorOnLine( jorge, AutonomousJorge.COLOR_SENSOR.BACK ) ) {
                jorge.drive.setAllPowers(PRESS_POWER); forwarding = true;
            }
        }
        runtime2.reset();
        jorge.drive.stopAll();


        // STEP 7
        //updateStatus( jorge, "Doing step 7 -- Backing up" );
        runtime.reset();
        while ( runtime.milliseconds() < INTERVAL_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
        jorge.drive.setAllPowers(-.2);
        runtime.reset();
        while ( runtime.milliseconds() < BACK_UP_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
        jorge.drive.stopAll();


//        updateStatus( jorge, "Doing step 8 -- Pressing multiple times if needed" );
//        noTimes--;
//        for ( int i = 0; i < noTimes; i++ ) {
//
//            while ( runtime2.milliseconds() < 6500 ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//
//            jorge.drive.setAllPowers(PRESS_POWER);
//            runtime.reset();
//            while ( runtime.milliseconds() < PRESS_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//            runtime2.reset();
//            jorge.drive.stopAll();
//
//            runtime.reset();
//            while ( runtime.milliseconds() < INTERVAL_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//
//            jorge.drive.setAllPowers(BACK_UP_POWER - .05);
//            runtime.reset();
//            while ( runtime.milliseconds() < BACK_UP_TIME ) { if ( !jorge.opMode.opModeIsActive() ) return; }
//            jorge.drive.stopAll();
//        }

        jorge.drive.stopAll();
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_L, Jorge.BEACON_UP );
        jorge.servos.setPosition( AutonomousJorge.BEACON_PRESSER_R, AutonomousJorge.BEACON_PRESSER_R_UP );

    }

    @Deprecated public static void FULL_PRESS_BEACON (AutonomousJorge jorge, AutonomousJorge.BEACON facingBeacon) throws InterruptedException {

        /*RED_PRESS_BEACON (jorge, 1);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        jorge.vuforia.checkOnBeacons(facingBeacon.index);
        if (jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_BLUE) {
            while (runtime.seconds() < 4) {jorge.opMode.telemetry.addData("Status", "Wrong color. Waiting to press..."); jorge.opMode.telemetry.update();}
            RED_PRESS_BEACON(jorge, 1);
        }*/

        if (!jorge.opMode.opModeIsActive()) return;

        jorge.vuforia.checkOnBeacons(facingBeacon.index);
        //if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_BLUE ) throw new Error("Woah, that shouldn't happen. Config = RED, BLUE");
        int noTimes = 1;
        if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_RED ) {} else
            if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_RED ) noTimes++;

        PRESS_BEACON( jorge, noTimes );

        jorge.vuforia.checkOnBeacons(facingBeacon.index);
        if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_BLUE ) {/* Press again. Code doesn't exist yet... */}

    }
    public static void FULL_PRESS_BEACON (AutonomousJorge jorge, AutonomousJorge.BEACON facingBeacon, SIDE side) throws InterruptedException {

        /*RED_PRESS_BEACON (jorge, 1);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        jorge.vuforia.checkOnBeacons(facingBeacon.index);
        if (jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_BLUE) {
            while (runtime.seconds() < 4) {jorge.opMode.telemetry.addData("Status", "Wrong color. Waiting to press..."); jorge.opMode.telemetry.update();}
            RED_PRESS_BEACON(jorge, 1);
        }*/

        if (!jorge.opMode.opModeIsActive()) return;

        jorge.vuforia.checkOnBeacons(facingBeacon.index);
        //if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_BLUE ) throw new Error("Woah, that shouldn't happen. Config = RED, BLUE");
//        int noTimes = 1;
//        if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_RED ) {} else
//        if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_RED ) noTimes++;
//
//        PRESS_BEACON( jorge, noTimes );

        if (side == SIDE.RED) {
            if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_RED ||
                    jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_BLUE )
            { RED_PRESS_BEACON(jorge, TurnToAngleTest.Direction.LEFT, false); } else
            if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_RED )
            {RED_PRESS_BEACON(jorge, TurnToAngleTest.Direction.RIGHT, false);}
            else { RED_PRESS_BEACON(jorge, TurnToAngleTest.Direction.RIGHT, false); }
            jorge.vuforia.checkOnBeacons(facingBeacon.index);
            if (jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_BLUE) {
                RED_PRESS_BEACON(jorge, TurnToAngleTest.Direction.RIGHT, false);
            }
        } else {
            if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_RED ||
                    jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_RED)
            { BLUE_PRESS_BEACON(jorge, TurnToAngleTest.Direction.LEFT, false); } else
            if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_BLUE )
            {BLUE_PRESS_BEACON(jorge, TurnToAngleTest.Direction.RIGHT);}
            else { BLUE_PRESS_BEACON(jorge, TurnToAngleTest.Direction.RIGHT, false); }
            jorge.vuforia.checkOnBeacons(facingBeacon.index);
            if (jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.RED_RED) {
                RED_PRESS_BEACON(jorge, TurnToAngleTest.Direction.LEFT, false);
            }
        }

        //jorge.vuforia.checkOnBeacons(facingBeacon.index);
        //if ( jorge.vuforia.getBeaconConfig(facingBeacon.index) == Vuforia2017Manager.BLUE_BLUE ) {/* Press again. Code doesn't exist yet... */}

    }

    public static void TURN_TO_RELATIVE_ANGLE (AutonomousJorge jorge, int angle, Path.Direction dir) {

        if (!jorge.opMode.opModeIsActive()) return;

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

        if (!jorge.opMode.opModeIsActive()) return;

        int distance = Math.round((inches / IN_DISTANCE_PER_1680) * 1680);
        boolean slowedDown = false;

        jorge.drive.setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jorge.drive.setAllModes(DcMotor.RunMode.RUN_TO_POSITION);
        jorge.drive.setAllTargetPositions(distance, maxSpeed);

        while (!compareTarget(jorge.drive.getAllPositions(), jorge.drive.getAllTargetPositions())) {
            if (!jorge.opMode.opModeIsActive()) return;
            if (!slowedDown &&
                    Math.abs(jorge.drive.getPosition("rb")) > Math.abs(Math.round(jorge.drive.getTargetPosition("rb")*(.6f)))
                    ) {
                jorge.drive.setAllPowers(maxSpeed/2); slowedDown = true; }
        }
        jorge.drive.stopAll();

    }

    public static void RED_FULL_BEACON (AutonomousJorge jorge) throws InterruptedException {

        GO_TO_WHITE_LINE(jorge);

        RED_STRAIGHTEN_ON_WHITE_LINE(jorge);

        FULL_PRESS_BEACON(jorge, AutonomousJorge.BEACON.B1, SIDE.RED);

    }

    public static void BLUE_FULL_BEACON (AutonomousJorge jorge) throws InterruptedException {

        GO_TO_WHITE_LINE(jorge);

        BLUE_STRAIGHTEN_ON_WHITE_LINE(jorge);

        FULL_PRESS_BEACON(jorge, AutonomousJorge.BEACON.B1, SIDE.BLUE);

    }

    public static void SHOOT_1 (AutonomousJorge jorge) {
        jorge.shoot(1);
    }

    public static void SHOOT_2 (AutonomousJorge jorge) {
        jorge.shoot(2);
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
        return squareRootDifference(jorge.colorSensors.getCRGB(sensor.port)) >= 70; }

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
