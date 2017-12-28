package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 12/27/17 at 6:25 PM by Akshay, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "Ggrabtest", group = "Iterative Opmode")
//@Disabled
public class Ggrabtest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo bl, br, tl, tr;
    private DcMotor r,l;
    private boolean
            a = false,
            b = false;

    private final double
            blo = 0.078,
            blc = 0.294,
            bro = 0.98,
            brc = 0.725,
            tlo = 0.98,
            tlc = 0.725,
            tro = 0.118,
            trc = 0.392;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        bl = hardwareMap.servo.get("bl");
        br = hardwareMap.servo.get("br");
        tl = hardwareMap.servo.get("tl");
        tr = hardwareMap.servo.get("tr");
        r = hardwareMap.dcMotor.get("r");
        l = hardwareMap.dcMotor.get("l");

        l.setDirection(DcMotorSimple.Direction.FORWARD);
        r.setDirection(DcMotorSimple.Direction.REVERSE);

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
        bl.setPosition(blo);
        br.setPosition(bro);
        tl.setPosition(tlo);
        tr.setPosition(tro);
        runtime.reset();
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double left = - gamepad1.right_stick_y + gamepad1.left_stick_x;
        double right = - gamepad1.right_stick_y - gamepad1.left_stick_x;

        if (gamepad1.right_trigger > 0.5) { left /= 4; right /= 4; }

        l.setPower(left);
        r.setPower(right);

        if (!a && gamepad1.a) { // bottom

            if (bl.getPosition() == blo) {
                //close
                bl.setPosition(blc);
                br.setPosition(brc);
            } else {
                //open
                bl.setPosition(blo);
                br.setPosition(bro);
            }

            a = true;
        } else if (a && !gamepad1.a) a = false;

        if (!b && gamepad1.b) { // top

            if (tl.getPosition() == tlo) {
                //close
                tl.setPosition(tlc);
                tr.setPosition(trc);
            } else {
                //open
                tl.setPosition(tlo);
                tr.setPosition(tro);
            }

            b = true;
        } else if (b && !gamepad1.b) b = false;

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        l.setPower(0);
        r.setPower(0);
    }

}
