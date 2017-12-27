package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Created on 12/3/17 at 3:12 PM by Akshay, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "PIControl", group = "Linear Opmode")
//@Disabled
public class PIControl extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        motor.getCurrentPosition();
        motor.setTargetPosition(3);
        int rps = 1120;
        DriveDistance(1, 1120);
        while (runtime.seconds() < 5)
        {

        }
        DriveDistance(1, 2240);



        waitForStart(); // Wait for user to press start
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }

    public void DriveForward(double power)
    {
        motor.setPower(power);
    }

    public void StopDriving()
    {
        motor.setPower(0);
    }

    public void DriveDistance(double power, int distance)
    {

        motor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor.setTargetPosition(distance);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveForward(double power);
        while (motor.isBusy())
        {

        }
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS)
    }
}
