package org.firstinspires.ftc.teamcode.robots.jorge.autonomous;

import android.graphics.Path;

import org.firstinspires.ftc.teamcode.robots.jorge.AutonomousJorge;

/**
 * Created by Chandler on 4/24/2017.
 */

public final class JorgeAutonomousMovement {

    // COLOR(START POS.), INITIAL, ENDING


    // RED1, POSITION 1, BEACON 1
    public static final float
            RED1_BEACON1_D1 = 10.12f,
            RED1_BEACON1_D2 = 53.67f;
    public static final int
            RED1_BEACON1_A1 = -40;

    // RED1, BEACON1, SHOOTING POSITION
    public static final float
            RED1_SHOOT_D1 = 2f;

    // RED1, SHOOTING POSITION, CENTER
    public static final float
            RED1_CENTER_D1 = -43.8f;
    public static final int
            RED1_CENTER_A1 = -23;

    // RED1, SHOOTING POSITION, CORNER
    public static final float
            RED1_CORNER_D1 = -31.37f;
    public static final int
            RED1_CORNER_A1 = 81;


    // RED2, POSITION 2, SHOOTING POSITION
    public static final float
            RED2_SHOOT_D1 = -6.1f,
            RED2_SHOOT_D2 = -15.86f;
    public static final int
            RED2_SHOOT_A1 = -43;

    // RED2, SHOOTING POSITION, CENTER

    public static final float
            RED2_CENTER_D1 = -43f;

    // RED2, SHOOTING POSITION, CENTER

    public static final float
            RED2_CORNER_D1 = -67.5f;
    public static final int
            RED2_CORNER_A1 = -49;


    // BLUE1, POSITION 1, BEACON 1
    public static final float
            BLUE1_BEACON1_D1 = 7.36f,
            BLUE1_BEACON1_D2 = 55.54f;
    public static final int
            BLUE1_BEACON1_A1 = 42;

    // BLUE1, BEACON1, SHOOTING POSITION
    public static final float
            BLUE1_SHOOT_D1 = 2f;

    // BLUE1, SHOOTING POSITION, CENTER
    public static final float
            BLUE1_CENTER_D1 = -50f;

    // BLUE1, SHOOTING POSITION, CORNER
    public static final float
            BLUE1_CORNER_D1 = -30.76f;
    public static final int
            BLUE1_CORNER_A1 = -78;


    // BLUE2, POSITION 2, SHOOTING POSITION
    public static final float
            BLUE2_SHOOT_D1 = -5.27f,
            BLUE2_SHOOT_D2 = -14.45f;
    public static final int
            BLUE2_SHOOT_A1 = 43;

    // BLUE2, SHOOTING POSITION, CENTER

    public static final float
            BLUE2_CENTER_D1 = -45.4f;

    // BLUE2, SHOOTING POSITION, CORNER

    public static final float
            BLUE2_CORNER_D1 = -67f;
    public static final int
            BLUE2_CORNER_A1 = 50;


    // RED
    public static void RED1_TO_BEACON ( AutonomousJorge jorge ) {

        driveIn(jorge, RED1_BEACON1_D1);

        turnToAngle(jorge, RED1_BEACON1_A1);

        driveIn(jorge, RED1_BEACON1_D2);

    }

    public static void RED1_TO_SHOOTING_POSITION ( AutonomousJorge jorge ) {

        driveIn(jorge, RED1_SHOOT_D1);

    }

    public static void RED1_TO_CENTER ( AutonomousJorge jorge ) {

        turnToAngle( jorge, RED1_CENTER_A1 );

        driveIn( jorge, RED1_CENTER_D1 );

    }

    public static void RED1_TO_CORNER ( AutonomousJorge jorge ) {

        turnToAngle( jorge, RED1_CORNER_A1 );

        driveIn( jorge, RED1_CORNER_D1 );

    }


    public static void RED2_TO_SHOOTING_POSITION ( AutonomousJorge jorge ) {

        driveIn( jorge, RED2_SHOOT_D1 );

        turnToAngle( jorge, RED2_SHOOT_A1 );

        driveIn( jorge, RED2_SHOOT_D2 );

    }

    public static void RED2_TO_CENTER ( AutonomousJorge jorge ) {

        driveIn( jorge, RED2_CENTER_D1 );

    }

    public static void RED2_TO_CORNER ( AutonomousJorge jorge ) {

        turnToAngle( jorge, RED2_CORNER_A1 );

        driveIn( jorge, RED2_CORNER_D1 );

    }


    public static void BLUE1_TO_BEACON ( AutonomousJorge jorge ) {

        driveIn( jorge, BLUE1_BEACON1_D1 );

        turnToAngle( jorge, BLUE1_BEACON1_A1 );

        driveIn( jorge, BLUE1_BEACON1_D2 );

    }

    public static void BLUE1_TO_SHOOTING_POSITION ( AutonomousJorge jorge ) {

        driveIn(jorge, BLUE1_SHOOT_D1);

    }

    public static void BLUE1_TO_CENTER ( AutonomousJorge jorge ) {

        driveIn( jorge, BLUE1_CENTER_D1 );

    }

    public static void BLUE1_TO_CORNER ( AutonomousJorge jorge ) {

        turnToAngle( jorge, BLUE1_CORNER_A1 );

        driveIn( jorge, BLUE1_CORNER_D1 );

    }


    public static void BLUE2_TO_SHOOTING_POSITION ( AutonomousJorge jorge ) {

        driveIn( jorge, BLUE2_SHOOT_D1 );

        turnToAngle( jorge, BLUE2_SHOOT_A1 );

        driveIn( jorge, BLUE2_SHOOT_D2 );

    }

    public static void BLUE2_TO_CENTER ( AutonomousJorge jorge ) {

        driveIn( jorge, BLUE2_CENTER_D1 );

    }

    public static void BLUE2_TO_CORNER ( AutonomousJorge jorge ) {

        turnToAngle( jorge, BLUE2_CORNER_A1 );

        driveIn( jorge, BLUE2_CORNER_D1 );

    }


    private static void turnToAngle (AutonomousJorge jorge, int angle) {
        if (angle > 0) {
            JorgeAutonomousFunctions.TURN_TO_RELATIVE_ANGLE(jorge, angle, Path.Direction.CW);
        } else {
            angle = Math.abs(angle);
            JorgeAutonomousFunctions.TURN_TO_RELATIVE_ANGLE(jorge, angle, Path.Direction.CCW);
        }
    }

    private static void driveIn ( AutonomousJorge jorge, float inches ) {
        if (inches > 0) { JorgeAutonomousFunctions.DRIVE_FORWARD_IN( jorge, inches, .8 );
        } else { JorgeAutonomousFunctions.DRIVE_FORWARD_IN( jorge, inches, -.8 ); }
    }

}
