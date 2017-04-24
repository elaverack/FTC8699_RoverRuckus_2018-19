package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotHandlers.EncodedRobotDrive;
import org.firstinspires.ftc.teamcode.robotHandlers.OptimizedGamepadRecorder;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotConfig;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotEncodedMotors;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotHandler;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotServos;
import org.firstinspires.ftc.teamcode.robotHandlers.StandardRobotDrive;

/**
 * Created by Chandler on 4/4/2017.
 */

public class Jorge extends RobotHandler {

    protected OpMode opMode;
    public EncodedRobotDrive drive;
    public RobotEncodedMotors auxMotors;
    public RobotServos servos;

    protected static final int
            SHOOTER_FIRE = 1680;
    public static final double
            RUN_PICK_UP     = -.8,
            STOP_PICK_UP    = 0,
            BEACON_DOWN     = .27,
            BEACON_UP       = .92,
            LOAD            = 0,
            UNLOAD          = 0.39;
    public static final String
            SHOOTER = "shoot",
            PICKUP = "pickup",
            BEACON_PRESSER = "beacon",
            LOADER = "load";

    protected Jorge(){}
    public Jorge (OpMode om) {
        this.opMode = om;
        //this.config = new RobotConfig(new StandardRobotDrive(opMode.hardwareMap));
        //config.getRobotDrive().setSideDirections(
        //        new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.RIGHT, StandardRobotDrive.SIDE.LEFT},
        //        new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD}
        //);

        this.config = new RobotConfig(4, RobotConfig.driveType.MECANUM, 2, 3);
        drive = new EncodedRobotDrive(opMode.hardwareMap);
        drive.setSideDirections(
                new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.RIGHT, StandardRobotDrive.SIDE.LEFT},
                new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD}
        );
        auxMotors = new RobotEncodedMotors(opMode.hardwareMap, new String[]{SHOOTER, PICKUP});
        servos = new RobotServos(opMode.hardwareMap, new String[]{BEACON_PRESSER, LOADER});

    }

    public void drive() {

        float straight = -opMode.gamepad1.right_stick_y;
        float strafe = opMode.gamepad1.right_stick_x;
        float rotate = opMode.gamepad1.left_stick_x;
        float slow = opMode.gamepad1.right_trigger;

        float powerRF = straight;
        float powerRB = straight;
        float powerLF = straight;
        float powerLB = straight;
        powerRF -= strafe;
        powerRB += strafe;
        powerLF += strafe;
        powerLB -= strafe;
        powerRF -= rotate;
        powerRB -= rotate;
        powerLF += rotate;
        powerLB += rotate;

        if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
        if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
        if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
        if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}

        if (slow > 0.25) {
            powerRF /= 4 * slow;
            powerRB /= 4 * slow;
            powerLF /= 4 * slow;
            powerLB /= 4 * slow;
        }

        drive.setPowers(new String[]{"rf", "rb", "lf", "lb"}, new double[]{powerRF, powerRB, powerLF, powerLB});
        doBeaconPresser(opMode.gamepad1.a);

        Gamepad gamepad2 = opMode.gamepad2;
        doPickUp(gamepad2.b);
        load(gamepad2.right_trigger, gamepad2.x);
        doShooting(gamepad2.a);

        opMode.telemetry.addData("CAM", auxMotors.getPosition(SHOOTER));
        opMode.telemetry.addData("CHECK", (auxMotors.getPosition(SHOOTER) > SHOOTER_FIRE || auxMotors.getPosition(SHOOTER) == SHOOTER_FIRE));
        opMode.telemetry.addData("SHOOTING N SET", shooting + ", " + set);

    }

    public void recordedDrive (OptimizedGamepadRecorder recorder) {

        float straight = -opMode.gamepad1.right_stick_y;
        float strafe = opMode.gamepad1.right_stick_x;
        float rotate = opMode.gamepad1.left_stick_x;
        float slow = opMode.gamepad1.right_trigger;

        recorder.doRecord(straight, rotate, strafe, slow);

        float powerRF = straight;
        float powerRB = straight;
        float powerLF = straight;
        float powerLB = straight;
        powerRF -= strafe;
        powerRB += strafe;
        powerLF += strafe;
        powerLB -= strafe;
        powerRF -= rotate;
        powerRB -= rotate;
        powerLF += rotate;
        powerLB += rotate;

        if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
        if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
        if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
        if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}

        if (slow > 0.25) {
            powerRF /= 4 * slow;
            powerRB /= 4 * slow;
            powerLF /= 4 * slow;
            powerLB /= 4 * slow;
        }

        drive.setPowers(new String[]{"rf", "rb", "lf", "lb"}, new double[]{powerRF, powerRB, powerLF, powerLB});

    }

    public void stop() {drive.stopAll(); auxMotors.stopAll();}

    protected boolean shooting = false;
    protected boolean set = false;
    protected void doShooting (boolean trigger) {

        if (!shooting && trigger) {
            shooting = true;
        }

        if (shooting) {
            if (!set) {
                //shooter.setTargetPosition(SHOOTER_FIRE);
                //shooter.setPower(.2);
                servos.setPosition(LOADER, LOAD);
                auxMotors.setTargetPosition(SHOOTER, SHOOTER_FIRE, .5);
                set = true;
            }
            if (/*(shooter.getCurrentPosition() > SHOOTER_FIRE || shooter.getCurrentPosition() == SHOOTER_FIRE)*/
                    (auxMotors.getPosition(SHOOTER) > SHOOTER_FIRE || auxMotors.getPosition(SHOOTER) == SHOOTER_FIRE) && set) {
                //shooter.setPower(0);
                //shooter.setTargetPosition(0);
                //shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                auxMotors.setPower(SHOOTER, 0);
                auxMotors.setTargetPosition(SHOOTER, 0, 0);
                auxMotors.setMode(SHOOTER, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auxMotors.setMode(SHOOTER, DcMotor.RunMode.RUN_TO_POSITION);
                shooting = false;
                set = false;
                servos.setPosition(LOADER, UNLOAD);
            }
        }

    }

    protected void load (float override, boolean button) {
        if (override > .25) {
            if (button) {
                servos.setPosition(LOADER, UNLOAD);
            } else {
                servos.setPosition(LOADER, LOAD);
            }
        }
    }

    protected boolean pickupToggled = false;
    protected boolean pickupOn = false;
    protected void doPickUp (boolean toggleButton) {
        if (!pickupToggled && toggleButton) {
            if (!pickupOn) {
                //pickUp.setPower(RUN_PICK_UP);
                auxMotors.setPower(PICKUP, RUN_PICK_UP);
                pickupOn = true;
                pickupToggled = true;
            } else {
                //pickUp.setPower(STOP_PICK_UP);
                auxMotors.setPower(PICKUP, STOP_PICK_UP);
                pickupOn = false;
                pickupToggled = true;
            }
        } else if (!toggleButton) {
            pickupToggled = false;
        }

    }

    protected boolean beaconToggled = false;
    protected void doBeaconPresser (boolean toggleButton) {
        if (!beaconToggled && toggleButton) {
            if (/*beacon_presser.getPosition()*/ servos.getPosition(BEACON_PRESSER) == BEACON_UP) {
                //beacon_presser.setPosition(BEACON_DOWN);
                servos.setPosition(BEACON_PRESSER, BEACON_DOWN);
            } else {
                //beacon_presser.setPosition(BEACON_UP);
                servos.setPosition(BEACON_PRESSER, BEACON_UP);
            }
            beaconToggled = true;
        } else if (!toggleButton) {
            beaconToggled = false;
        }
    }

}
