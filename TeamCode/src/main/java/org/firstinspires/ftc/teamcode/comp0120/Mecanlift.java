package org.firstinspires.ftc.teamcode.comp0120;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Created on 12/27/17 at 6:25 PM by Akshay, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "mecanlift", group = "Iterative Opmode")
@Disabled
public class Mecanlift extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo bl, br, tl, tr;
    private DcMotor rf, rb, lf, lb;
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

    @Override
    public void init() {
        bl = hardwareMap.servo.get("bl");
        br = hardwareMap.servo.get("br");
        tl = hardwareMap.servo.get("tl");
        tr = hardwareMap.servo.get("tr");

        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() {
        bl.setPosition(blo);
        br.setPosition(bro);
        tl.setPosition(tlo);
        tr.setPosition(tro);
        runtime.reset();
    }

    @Override
    public void loop() {

        float straight = -gamepad1.right_stick_y;
        float strafe = gamepad1.right_stick_x;
        float rotate = gamepad1.left_stick_x;
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

        if (gamepad1.right_trigger > 0.5) { powerLB /= 4; powerLF /= 4; powerRB /= 4; powerRF /= 4; }

        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);


        if (!a && gamepad1.a) { // bottom gripper
            if (bl.getPosition() == blo) { bl.setPosition(blc); br.setPosition(brc); }  // close
            else { bl.setPosition(blo); br.setPosition(bro); } // open
            a = true;
        } else if (a && !gamepad1.a) a = false;
        if (!b && gamepad1.b) { // top gripper
            if (tl.getPosition() == tlo) { tl.setPosition(tlc); tr.setPosition(trc); } // close
            else { tl.setPosition(tlo); tr.setPosition(tro); } // open
            b = true;
        } else if (b && !gamepad1.b) b = false;


        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    @Override
    public void stop() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

}
