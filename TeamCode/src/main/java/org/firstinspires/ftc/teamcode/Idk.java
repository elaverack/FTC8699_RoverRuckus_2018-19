package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Created on 11/29/17 at 7:25 PM by Akshay, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "Idk", group = "Linear Opmode")
//@Disabled
public class Idk extends LinearOpMode {

    // Declare OpMode members.
    DcMotor motor;
    private ElapsedTime runtime = new ElapsedTime();
    private final double ticksPerRevolution = 1000;
    private double prevTime;
    private int prevEncoderPosition;
    private final double drivePidKp = 1;    // Tuning variable for PID.
    private final double drivePidTi = 1.0;  // Eliminate integral error in 1 sec.
    private final double drivePidTd = 0.1;  // Account for error in 0.1 sec.
    // Protect against integral windup by limiting integral term.
    private final double maxSpeed = 1;  // Limit to max speed.

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
         prevTime = 0;
        prevEncoderPosition = motor.getCurrentPosition();

        waitForStart(); // Wait for user to press start
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double deltaTime = time - prevTime;
            double speed = (motor.getCurrentPosition() - prevEncoderPosition) / deltaTime;

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            prevTime = time;
            prevEncoderPosition = motor.getCurrentPosition();

        }
    }

}
