package org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions;

import com.qualcomm.ftccommon.DbgLog;

import java.text.MessageFormat;

/**
 * Created by Chandler on 1/16/2016.
 */
public class DirectionHandler {

    MotorHandler DriveTrain;
    String[] leftMotors;
    String[] rightMotors;
    final double radiusOfRobot = 9.18;

    public DirectionHandler(MotorHandler DriveTrain, String[] motorNames /*Note that motor names should be as follows: Right Front, Right Back, Left Front, Left Back*/) {
        this.DriveTrain = DriveTrain;
        leftMotors = new String[]{motorNames[2], motorNames[3]};
        rightMotors = new String[]{motorNames[0], motorNames[1]};
    }

    public void turnByAngle(int angle, boolean clockwise) {
        double f = angle * Math.PI;
        double radians = f / 180;
        DbgLog.msg("For turn angle " + angle + ", radians = " + radians);
        double distancePerWheel = radians * radiusOfRobot;
        DbgLog.msg(MessageFormat.format("turnByAngle: Distance per wheel calculated to be {0}", distancePerWheel));
        if (clockwise) {
            DriveTrain.rightGoal = DriveTrain.moveDistanceEncoder((float) -distancePerWheel, rightMotors);
            DriveTrain.leftGoal = DriveTrain.moveDistanceEncoder((float) distancePerWheel, leftMotors);
        } else {
            DriveTrain.rightGoal = DriveTrain.moveDistanceEncoder((float) distancePerWheel, rightMotors);
            DriveTrain.leftGoal = DriveTrain.moveDistanceEncoder((float) -distancePerWheel, leftMotors);
        }
    }

    public void turnLeft() {
        turnByAngle(90, false);
    }

    public void turnRight() {
        turnByAngle(90, true);
    }

    public long turnByAngleEstimate(int angle, boolean clockwise) {
        double radians = ((angle / 180) * Math.PI);
        double distancePerWheel = radians * radiusOfRobot;
        if (clockwise) {
            DriveTrain.setMotorsPower(leftMotors, 1);
            DriveTrain.setMotorsPower(rightMotors, -1);


            return DriveTrain.moveDistanceEstimation(distancePerWheel);

        } else {
            DriveTrain.setMotorsPower(leftMotors, -1);
            DriveTrain.setMotorsPower(rightMotors, 1);


            return DriveTrain.moveDistanceEstimation(distancePerWheel);

        }
    }

    public long turnLeftEstimate() {
        return turnByAngleEstimate(90, false);
    }

    public long turnRightEstimate() {
        return turnByAngleEstimate(90, true);
    }

    public int manualEncoderTurn(int angle, boolean clockwise, int wheelDiameter) {
        double radians = ((angle / 180) * Math.PI);
        double distance = radians * radiusOfRobot;
        float wheelCircumference = (float) Math.PI * wheelDiameter;
        return ((int) (distance / wheelCircumference) * 1680);
    }
}
