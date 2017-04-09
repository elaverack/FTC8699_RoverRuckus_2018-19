package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Chandler on 2/19/2017.
 */

public class StandardRobotDrive extends RobotMotors {

    public enum SIDE {
        LEFT(new String[]{"lf", "lb"}), RIGHT(new String[]{"rf", "rb"});

        String[] names;

        SIDE (String[] sideNames) {
            names = sideNames;
        }
    }

    public StandardRobotDrive(HardwareMap hm) {
        hardwareMap = hm;
        addMotors(new String[]{"rf", "rb", "lf", "lb"});
    }

    public void setSidePower (SIDE side, double power) {setPowers(side.names, power);}
    public void setSidePowers (SIDE[] sides, double[] powers) {
        //setPowers(side.names, power);
        if (sides.length != powers.length) return;
        for (int i = 0; i < sides.length; i++) {
            setSidePower(sides[i], powers[i]);
        }
    }

    public void setSideDirection (SIDE side, DcMotorSimple.Direction direction) {setDirections(side.names, direction);}
    public void setSideDirections (SIDE[] sides, DcMotorSimple.Direction[] directions) {
        //setDirections(side.names, direction);
        if (sides.length != directions.length) return;
        for (int i = 0; i < sides.length; i++) {
            setSideDirection(sides[i], directions[i]);
        }
    }

    /*public void forwardOneFullRotation (int motorGearBox, double power, LinearOpMode opMode) {
        if (!hasEncoders) return;
        int encoder = motorGearBox*28; // Change per motor
        setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllModes(DcMotor.RunMode.RUN_TO_POSITION);
        setAllTargetPositions(encoder, power);
        boolean Break = false;
        while (!Break) {
            if (!opMode.opModeIsActive()) {Break = true; continue;}
            boolean atPosition = true;
            for (String motorName:motorNames) {int currentPos = getPosition(motorName);
                atPosition = (currentPos > encoder || currentPos == encoder);
            }
            Break = atPosition;
        }
        setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void forwardSomeFeetWithEncoders (double feet, int motorGearBox, double radiusINCHES, double power, LinearOpMode opMode) {
        if (!hasEncoders) return;
        int encoder = (int)Math.round((motorGearBox*28)*(feet/(((radiusINCHES*2)*Math.PI)/12))); // Calculation of motor encoder values. Takes encoder value for one rotation and multiplies it by the distance over the circumference of the wheels
        setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllModes(DcMotor.RunMode.RUN_TO_POSITION);
        setAllTargetPositions(encoder, power);
        boolean Break = false;
        while (!Break) {
            if (!opMode.opModeIsActive()) {Break = true; continue;}
            boolean atPosition = true;
            for (String motorName:motorNames) {int currentPos = getPosition(motorName);
                atPosition = (currentPos > encoder || currentPos == encoder);
            }
            Break = atPosition;
        }
        setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }*/

}
