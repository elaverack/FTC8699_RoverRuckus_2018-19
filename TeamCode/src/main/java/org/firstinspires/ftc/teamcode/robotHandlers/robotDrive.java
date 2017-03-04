package org.firstinspires.ftc.teamcode.robotHandlers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedHashMap;

/**
 * Created by Chandler on 2/19/2017.
 */

public class RobotDrive {

    private LinkedHashMap<String, DcMotor> driveMotors = new LinkedHashMap<>();
    private boolean hasEncoders = false; //TODO: Mod so that only a portion need encoders
    private ControlConfig gamepadConfig = null;
    private boolean doSlow = false;
    private HardwareMap hardwareMap;
    private String[] motorNames;
    private ElapsedTime runtime;
    public static enum SIDE {LEFT, RIGHT}
    //TODO: Make circumference variable here

    public RobotDrive(HardwareMap hm, ControlConfig config) {
        hardwareMap = hm;
        //TODO: Make more flexible
        motorNames = new String[]{"rf", "rb", "lf", "lb"};
        for (String name:motorNames) {driveMotors.put(name, hardwareMap.dcMotor.get(name));}
        gamepadConfig = config;
        doSlow = false;
        this.hasEncoders = false;
        runtime = new ElapsedTime();
    }
    public RobotDrive(HardwareMap hm, boolean hasEncoders, ControlConfig config) {
        hardwareMap = hm;
        //TODO: Make more flexible
        motorNames = new String[]{"rf", "rb", "lf", "lb"};
        for (String name:motorNames) {driveMotors.put(name, hardwareMap.dcMotor.get(name));}
        gamepadConfig = config;
        doSlow = false;
        this.hasEncoders = hasEncoders;
        runtime = new ElapsedTime();
    }
    public RobotDrive(HardwareMap hm, ControlConfig config, boolean slow) {
        hardwareMap = hm;
        //TODO: Make more flexible
        motorNames = new String[]{"rf", "rb", "lf", "lb"};
        for (String name:motorNames) {driveMotors.put(name, hardwareMap.dcMotor.get(name));}
        gamepadConfig = config;
        doSlow = slow;
        this.hasEncoders = false;
        runtime = new ElapsedTime();
    }
    public RobotDrive(HardwareMap hm, boolean hasEncoders, ControlConfig config, boolean slow) {
        hardwareMap = hm;
        //TODO: Make more flexible
        motorNames = new String[]{"rf", "rb", "lf", "lb"};
        for (String name:motorNames) {driveMotors.put(name, hardwareMap.dcMotor.get(name));}
        gamepadConfig = config;
        doSlow = slow;
        this.hasEncoders = hasEncoders;
        runtime = new ElapsedTime();
    }

    public DcMotor getMotor (String motorName) {return driveMotors.get(motorName);}
    public LinkedHashMap<String, DcMotor> getMotors (String[] motorNames) {
        LinkedHashMap<String, DcMotor> Return = new LinkedHashMap<>();
        for (String motorName:motorNames) {Return.put(motorName, getMotor(motorName));}
        return Return;
    }
    public LinkedHashMap<String, DcMotor> getAllMotors () {return driveMotors;}

    public void drive (Gamepad gamepad) {
        if (gamepadConfig == null) return;
        if (gamepadConfig == ControlConfig.MECHANUM) {calculateValuesandDrive(-gamepad.left_stick_y, gamepad.right_stick_x, gamepad.left_stick_x); return;}
        if (gamepadConfig == ControlConfig.TANK) {
            float powerRF = -gamepad.right_stick_y, powerRB = -gamepad.right_stick_y, powerLF = -gamepad.left_stick_y, powerLB = -gamepad.left_stick_y;
            if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
            if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
            if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
            if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}
            setPower("rf", powerRF); setPower("rb", powerRB); setPower("lf", powerLF); setPower("lb", powerLB);
        }
        //TODO: Add code in-case of odd controller config
    }
    public void drive (Gamepad gamepad, float slow) {
        if (gamepadConfig == null) return;
        if (gamepadConfig == ControlConfig.MECHANUM) {
            if (doSlow) {calculateValuesandDrive(-gamepad.left_stick_y, gamepad.right_stick_x, gamepad.left_stick_x, slow);}
            else {calculateValuesandDrive(-gamepad.left_stick_y, gamepad.right_stick_x, gamepad.left_stick_x);}
            return;
        }

        if (gamepadConfig == ControlConfig.TANK) {
            if (doSlow) {doSlowandDrive(-gamepad.left_stick_y, -gamepad.right_stick_y, slow);}
            else {
                float powerRF = -gamepad.right_stick_y, powerRB = -gamepad.right_stick_y, powerLF = -gamepad.left_stick_y, powerLB = -gamepad.left_stick_y;
                if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
                if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
                if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
                if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}
                setPower("rf", powerRF); setPower("rb", powerRB); setPower("lf", powerLF); setPower("lb", powerLB);
            }
        }
        //TODO: Add code in-case of odd controller config
    }

    public void calculateValuesandDrive (float straight, float rotate, float strafe, float slow) {

        double powerRF, powerRB, powerLF, powerLB;

        powerRF = straight; powerRB = straight; powerLF = straight; powerLB = straight;
        powerRF -= strafe; powerRB += strafe; powerLF += strafe; powerLB -= strafe;
        powerRF -= rotate; powerRB -= rotate; powerLF += rotate; powerLB += rotate;

        if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
        if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
        if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
        if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}
        if (slow > 0.25) {powerRF /= 4 * slow; powerRB /= 4 * slow; powerLF /= 4 * slow; powerLB /= 4 * slow;}

        setPower("rf", powerRF); setPower("rb", powerRB); setPower("lf", powerLF); setPower("lb", powerLB);

    }
    public void doSlowandDrive (float left, float right, float slow) {

        double powerRF, powerRB, powerLF, powerLB;

        powerRF = right; powerRB = right; powerLF = left; powerLB = left;

        if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
        if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
        if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
        if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}
        if (slow > 0.25) {powerRF /= 4 * slow; powerRB /= 4 * slow; powerLF /= 4 * slow; powerLB /= 4 * slow;}

        setPower("rf", powerRF); setPower("rb", powerRB); setPower("lf", powerLF); setPower("lb", powerLB);

    }
    public void calculateValuesandDrive (float straight, float rotate, float strafe) {

        double powerRF, powerRB, powerLF, powerLB;

        powerRF = straight; powerRB = straight; powerLF = straight; powerLB = straight;
        powerRF -= strafe; powerRB += strafe; powerLF += strafe; powerLB -= strafe;
        powerRF -= rotate; powerRB -= rotate; powerLF += rotate; powerLB += rotate;

        if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
        if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
        if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
        if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}

        setPower("rf", powerRF); setPower("rb", powerRB); setPower("lf", powerLF); setPower("lb", powerLB);

    }

    public void setPower (String motorName, double power) {driveMotors.get(motorName).setPower(power);}
    public void setPowers (String[] motorNames, double power) {
        for (String motorName:motorNames) {setPower(motorName, power);}
    }
    public void setPowers (String[] motorNames, double[] powers) {
        if (motorNames.length != powers.length) return;
        for (int i = 0; i < motorNames.length; i++) {setPower(motorNames[i], powers[i]);}
    }
    public void setAllPowers (double power) {setPowers(motorNames, power);}

    public void brake(String motorName) {setPower(motorName, 0);}
    public void brake(String[] motorNames) {for (String motorName:motorNames) {brake(motorName);}}
    public void brakeAll() {setAllPowers(0);}

    public void setMode (String motorName, DcMotor.RunMode mode) {driveMotors.get(motorName).setMode(mode);}
    public void setModes (String[] motorNames, DcMotor.RunMode mode) {
        for (String motorName:motorNames) {setMode(motorName, mode);}
    }
    public void setModes (String[] motorNames, DcMotor.RunMode[] modes) {
        if (motorNames.length != modes.length) return;
        for (int i = 0; i < motorNames.length; i++) {setMode(motorNames[i], modes[i]);}
    }
    public void setAllModes (DcMotor.RunMode mode) {
        setModes(motorNames, mode);
    }

    public void setTargetPosition (String motorName, int position, double power) {
        if (!hasEncoders) return;
        driveMotors.get(motorName).setTargetPosition(position);
        setPower(motorName, power);
    }
    public void setTargetPositions (String[] motorNames, int position, double power) {
        if (!hasEncoders) return;
        for (String motorName:motorNames) {
            setTargetPosition(motorName, position, power);
        }
    }
    public void setTargetPositions (String[] motorNames, int[] positions, double[] powers) {
        if (!hasEncoders || (motorNames.length != positions.length || motorNames.length != powers.length)) return;
        for (int i = 0; i < motorNames.length; i++) {
            setTargetPosition(motorNames[i], positions[i], powers[i]);
        }
    }
    public void setAllTargetPositions (int position, double power) {
        setTargetPositions(motorNames, position, power);
    }

    public int getPosition (String motorName) {
        if (!hasEncoders) return 0;
        return driveMotors.get(motorName).getCurrentPosition();
    }
    public int[] getPositions (String[] motorNames) {
        if (!hasEncoders) return null;
        int[] Return = new int[motorNames.length];
        for (int i = 0; i < motorNames.length; i++) {
            Return[i] = getPosition(motorNames[i]);
        }
        return Return;
    }
    public int[] getAllPositions () {return getPositions(motorNames);}

    public void setDirection (String motorName, DcMotorSimple.Direction direction) {driveMotors.get(motorName).setDirection(direction);}
    public void setDirections (String[] motorNames, DcMotorSimple.Direction direction) {
        for (String motorName:motorNames) {setDirection(motorName, direction);}
    }
    public void setDirections (String[] motorNames, DcMotorSimple.Direction[] directions) {
        if (motorNames.length != directions.length) return;
        for (int i = 0; i < motorNames.length; i++) {setDirection(motorNames[i], directions[i]);}
    }
    public void setAllDirections (DcMotorSimple.Direction direction) {setDirections(motorNames, direction);}
    public void setSideDirection (SIDE side, DcMotorSimple.Direction direction) {setDirections(getSideNames(side), direction);}

    public String[] getSideNames (SIDE side) {
        // Note: Motor names go {right side names, left side names}
        int sideSize;
        String[] Return;
        switch (side) {
            case LEFT:
                sideSize = (motorNames.length)/2; Return = new String[sideSize];
                for (int i = 0; i < sideSize; i++) {Return[i] = motorNames[i+sideSize];}
                return Return;
            case RIGHT:
                sideSize = (motorNames.length)/2; Return = new String[sideSize];
                for (int i = 0; i < sideSize; i++) {Return[i] = motorNames[i];}
                return Return;
            default:
                return null;
        }
    }

    public void forwardOneFullRotation (int motorGearBox, double power, LinearOpMode opMode) {
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
    }

}
