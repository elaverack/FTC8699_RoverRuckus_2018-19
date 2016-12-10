package org.firstinspires.ftc.teamcode.barebones_testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Vector;

// Created on 12/9/2016 at 9:13 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.barebones_testing

@TeleOp(name = "bt_servoMovementRecord", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class bt_servoMovementRecord extends OpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Servo servo;
    TouchSensor sensor;

    //Vector save = new Vector();

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // Set to REVERSE if using AndyMark motors
        // Set to FORWARD if using AndyMark motors

        servo = hardwareMap.servo.get("servo");
        sensor = hardwareMap.touchSensor.get("sensor");

    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {


    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {

        runtime.reset();

        servo.setPosition(0);

    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)

        if (sensor.isPressed()) {
            servo.setPosition(1);
        } else {
            servo.setPosition(0);
        }

        telemetry.addData("Servo Position", servo.getPosition());

        //save.addElement(new Object[]{runtime.toString(), servo.getPosition()});

    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

        // eg: Set all motor powers to 0

        servo.setPosition(0);

    }

}
