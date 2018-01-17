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
    private DcMotor r,l,lift;
    private boolean
            a       = false,
            b       = false,
            y       = false,
            up      = false,
            down    = false;

    private final double
            blo     = 0.078,
            blc     = 0.294,
            bro     = 0.98,
            brc     = 0.725,
            tlo     = 0.98,
            tlc     = 0.725,
            tro     = 0.118,
            trc     = 0.392,
            liftS   = .55,
            liftDS  = .25;

    private final int
            lift0 = 0,
            lift1 = 3000,
            lift2 = 5330,
            thres = 10;


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
        lift = hardwareMap.dcMotor.get("lift");

        l.setDirection(DcMotorSimple.Direction.FORWARD);
        r.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

//        if (!a && gamepad1.a) { // bottom
//
//            if (bl.getPosition() == blo) {
//                //close
//                bl.setPosition(blc);
//                br.setPosition(brc);
//            } else {
//                //open
//                bl.setPosition(blo);
//                br.setPosition(bro);
//            }
//
//            a = true;
//        } else if (a && !gamepad1.a) a = false;
//
//        if (!b && gamepad1.b) { // top
//
//            if (tl.getPosition() == tlo) {
//                //close
//                tl.setPosition(tlc);
//                tr.setPosition(trc);
//            } else {
//                //open
//                tl.setPosition(tlo);
//                tr.setPosition(tro);
//            }
//
//            b = true;
//        } else if (b && !gamepad1.b) b = false;

        tr.setPosition(tro + (trc - tro) * gamepad1.left_trigger);
        tl.setPosition(tlo + (tlc - tlo) * gamepad1.left_trigger);
        br.setPosition(bro + (brc - bro) * gamepad1.right_trigger);
        bl.setPosition(blo + (blc - blo) * gamepad1.right_trigger);

        // Lift
        if (!up && gamepad1.dpad_up) {
            switch (lift.getTargetPosition()) {
                case lift0:
                    lift.setTargetPosition(lift1);
                    lift.setPower(liftS);
                    break;
                case lift1:
                    lift.setTargetPosition(lift2);
                    lift.setPower(liftS);
                    break;
                case lift2:
                    lift.setTargetPosition(lift1);
                    lift.setPower(-liftS);
                    break;
            }
            up = true;
        } else if (up && !gamepad1.dpad_up) up = false;

        if (!down && gamepad1.dpad_down) {
            lift.setTargetPosition(lift0);
            lift.setPower(-liftS);
        } else if (!gamepad1.dpad_down) down = false;

        if (lift.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                (lift.getCurrentPosition() < lift.getTargetPosition() + thres &&
                        lift.getCurrentPosition() > lift.getTargetPosition() - thres)) {
            lift.setPower(0);
        }

        if (!y && gamepad1.y) { // First press of y
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(liftDS);
            y = true;
        } else if (y && gamepad1.y) { // Holding y
            lift.setPower(liftDS);
        } else if (y) { // Release of y
            lift.setPower(0);
            y = false;
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (!a && gamepad1.a) { // First press of a
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(-liftDS);
            a = true;
        } else if (a && gamepad1.a) { // Holding a
            lift.setPower(-liftDS);
        } else if (a) { // Release of a
            lift.setPower(0);
            a = false;
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        l.setPower(0);
        r.setPower(0);
        lift.setPower(0);
    }

}
