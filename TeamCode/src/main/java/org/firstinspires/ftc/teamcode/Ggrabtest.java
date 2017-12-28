package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 12/27/17 at 6:25 PM by Akshay, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "Ggrabtest", group = "Iterative Opmode")
//@Disabled
public class Ggrabtest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo bl, br, tl, tr;
    private int sw = 1;
    private int sw2 = 1;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        bl = hardwareMap.servo.get(bl);
        br = hardwareMap.servo.get(br);
        tl = hardwareMap.servo.get(tl);
        tr = hardwareMap.servo.get(tr);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        bl.setPosition(0.294);
        br.setPosition(0.725);
        tl.setPosition(0.725);
        tr.setPosition(0.392);
        runtime.reset();
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.a())    //bottom
        {
            if (sw == 1) //open it
            {
                bl.setPosition(0.078);
                br.setPosition(0.98);
                sw *= -1
            }
            else             //close it
            {
                bl.setPosition(0.294);
                br.setPosition(0.725);
                sw *= -1;
            }
        }

        if (gamepad1.b())        //top
        {
            if (sw2 == 1)     //open it
            {
                tl.setPosition(0.98);
                tr.setPosition(0.118);
                sw2 *= -1
            }
            else              //close it
            {
                tl.setPosition(0.392);
                tr.setPosition(0.723);
                sw2 *= -1;
            }
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
