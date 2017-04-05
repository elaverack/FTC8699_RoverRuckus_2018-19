package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotHandlers.RobotConfig;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotHandler;
import org.firstinspires.ftc.teamcode.robotHandlers.StandardRobotDrive;

/**
 * Created by Chandler on 4/4/2017.
 */

public class Jorge extends RobotHandler {

    private OpMode opMode;

    public Jorge (OpMode om, HardwareMap hm) {
        this.config = new RobotConfig(new StandardRobotDrive(hm));
        this.opMode = om;
        config.getRobotDrive().setSideDirections(
                new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.RIGHT, StandardRobotDrive.SIDE.LEFT},
                new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD}
        );
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

        StandardRobotDrive drive = config.getRobotDrive();
        drive.setPowers(new String[]{"rf", "rb", "lf", "lb"}, new double[]{powerRF, powerRB, powerLF, powerLB});

    }

}
