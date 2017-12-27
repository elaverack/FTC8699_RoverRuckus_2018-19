package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;

// Created on 11/8/17 at 6:55 PM by Akshay, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "showing", group = "Linear Opmode")
//@Disabled
public class showing extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motor;

    @Override
    public void runOpMode() {

        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart(); // Wait for user to press start
        runtime.reset();

        motor.setPower(0.99);

        double circumfrence = 4 * Math.PI;
        double revs = 48/circumfrence;

        if (motor.getCurrentPosition() == revs * 1120)
        {
            motor.setPower(0);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
