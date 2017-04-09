package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Chandler on 4/8/2017.
 */

public class EncodedRobotDrive extends RobotEncodedMotors {

    public EncodedRobotDrive(HardwareMap hm) {
        hardwareMap = hm;
        addMotors(new String[]{"rf", "rb", "lf", "lb"});
    }

    public void setSidePower (StandardRobotDrive.SIDE side, double power) {setPowers(side.names, power);}
    public void setSidePowers (StandardRobotDrive.SIDE[] sides, double[] powers) {
        //setPowers(side.names, power);
        if (sides.length != powers.length) return;
        for (int i = 0; i < sides.length; i++) {
            setSidePower(sides[i], powers[i]);
        }
    }

    public void setSideDirection (StandardRobotDrive.SIDE side, DcMotorSimple.Direction direction) {setDirections(side.names, direction);}
    public void setSideDirections (StandardRobotDrive.SIDE[] sides, DcMotorSimple.Direction[] directions) {
        //setDirections(side.names, direction);
        if (sides.length != directions.length) return;
        for (int i = 0; i < sides.length; i++) {
            setSideDirection(sides[i], directions[i]);
        }
    }

}
