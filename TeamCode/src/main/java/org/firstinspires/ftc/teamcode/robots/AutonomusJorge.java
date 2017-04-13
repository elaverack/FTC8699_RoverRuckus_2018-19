package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotHandlers.EncodedRobotDrive;
import org.firstinspires.ftc.teamcode.robotHandlers.MultiplexedColorSensors;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotConfig;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotEncodedMotors;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotHandler;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotServos;
import org.firstinspires.ftc.teamcode.robotHandlers.StandardRobotDrive;

/**
 * Created by Chandler on 4/4/2017.
 */

public class AutonomusJorge extends Jorge {

    public MultiplexedColorSensors colorSensors;
    public LinearOpMode opMode;

    private final int
            NUMBER_OF_SENSORS   = 2;

    public enum COLOR_SENSOR {
        MIDDLE(0), BACK(1);

        public final int port;

        COLOR_SENSOR(int port) {
            this.port = port;
        }
    }

    public AutonomusJorge(LinearOpMode om) {
        this.opMode = om;
        this.config = new RobotConfig(4, RobotConfig.driveType.MECANUM, 2, 3);
        drive = new EncodedRobotDrive(opMode.hardwareMap);
        drive.setSideDirections(
                new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.RIGHT, StandardRobotDrive.SIDE.LEFT},
                new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD}
        );
        auxMotors = new RobotEncodedMotors(opMode.hardwareMap, new String[]{SHOOTER, PICKUP});
        servos = new RobotServos(opMode.hardwareMap, new String[]{BEACON_PRESSER, LOADER});

        colorSensors = new MultiplexedColorSensors(opMode.hardwareMap, "mux", "ada", NUMBER_OF_SENSORS, MultiplexedColorSensors.ATIME.FASTEST, MultiplexedColorSensors.GAIN._16X);
    }

}
