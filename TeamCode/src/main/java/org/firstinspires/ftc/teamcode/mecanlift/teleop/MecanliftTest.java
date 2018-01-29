package org.firstinspires.ftc.teamcode.mecanlift.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecanlift.controller.Lift;
import org.firstinspires.ftc.teamcode.mecanlift.controller.ToggleServo;

// Created on 1/25/2018 at 2:35 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

@TeleOp(name = "Mecanlift Test", group = "Iterative Opmode")
//@Disabled
public class MecanliftTest extends OpMode {

    private static final double     // Grabber servo positions
            blo = 0.078,            // Bottom left open
            blc = 0.310,            // Bottom left close
            bro = 0.98,             // Bottom right open
            brc = 0.670,            // Bottom right close
            tlo = 0.98,             // Top left open
            tlc = 0.725,            // Top left close
            tro = 0.118,            // Top right open
            trc = 0.392;            // Top right close

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rf, rb, lf, lb;
    private ToggleServo tl, tr, bl, br;
    private Lift lift;
    private boolean[] dup, ddown, y, x, a, b; // NOTE: Order is { gamepad1, gamepad2 }

    @Override
    public void init() {

        /** DRIVE */
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);

        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /** LIFT */
        lift = new Lift(hardwareMap.dcMotor.get("lift"));

        /** GRABBER */
        bl = new ToggleServo(hardwareMap.servo.get("bl"), blo, blc);
        br = new ToggleServo(hardwareMap.servo.get("br"), bro, brc);
        tl = new ToggleServo(hardwareMap.servo.get("tl"), tlo, tlc);
        tr = new ToggleServo(hardwareMap.servo.get("tr"), tro, trc);

        /** TWO CONTROLLER SUPPORT */
        dup     = new boolean[]{ false, false };
        ddown   = new boolean[]{ false, false };
        x       = new boolean[]{ false, false };
        y       = new boolean[]{ false, false };
        a       = new boolean[]{ false, false };
        b       = new boolean[]{ false, false };

        telemetry.setCaptionValueSeparator(" | ");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() {
        lift.start(); bl.open(); br.open(); tl.open(); tr.open();
        hardwareMap.servo.get("jewel").setPosition(1);
        runtime.reset();
    }

    @Override
    public void loop() {

        /** LIFT */
        lift.run(do_dup(), do_ddown(), do_y(), do_x());

        /** GRABBER */
        boolean a = do_a(), b = do_b();
        bl.tob(a);
        br.tob(a);
        tl.tob(b);
        tr.tob(b);

        /** MECANUM WHEELS */
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

        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Joys", String.format("y: %1$s, x: %2$s, r: %3$s",
                round(-gamepad1.right_stick_y),
                round(gamepad1.right_stick_x),
                round(gamepad1.left_stick_x)
        ));
        telemetry.addData("Pows", String.format("rf: %1$s, rb: %2$s, lf: %3$s, lb: %4$s",
                round(powerRF), round(powerRB), round(powerLF), round(powerLB)
        ));
    }

    @Override
    public void stop() { rf.setPower(0); rb.setPower(0); lf.setPower(0); lb.setPower(0); lift.stop(); }

    private static float round(float val) { return ((float)Math.round(val*1000f))/1000f; }

    private boolean do_dup () {
        if (gamepad1.dpad_up && !dup[0]) {
            dup[0] = true;
            return true;
        } else if (!gamepad1.dpad_up && dup[0]) dup[0] = false;
        if (gamepad2.dpad_up && !dup[1]) {
            dup[1] = true;
            return true;
        } else if (!gamepad2.dpad_up && dup[1]) dup[1] = false;
        return false;
    }
    private boolean do_ddown () {
        if (gamepad1.dpad_down && !ddown[0]) {
            ddown[0] = true;
            return true;
        } else if (!gamepad1.dpad_down && ddown[0]) ddown[0] = false;
        if (gamepad2.dpad_down && !ddown[1]) {
            ddown[1] = true;
            return true;
        } else if (!gamepad2.dpad_down && ddown[1]) ddown[1] = false;
        return false;
    }
    private boolean do_y () { return gamepad1.y || gamepad2.y; }
    private boolean do_x () { return gamepad1.x || gamepad2.x; }
    private boolean do_a () {
        if (gamepad1.a && !a[0]) {
            a[0] = true;
            return true;
        } else if (!gamepad1.a && a[0]) a[0] = false;
        if (gamepad2.a && !a[1]) {
            a[1] = true;
            return true;
        } else if (!gamepad2.a && a[1]) a[1] = false;
        return false;
    }
    private boolean do_b () {
        if (gamepad1.b && !b[0]) {
            b[0] = true;
            return true;
        } else if (!gamepad1.b && b[0]) b[0] = false;
        if (gamepad2.b && !b[1]) {
            b[1] = true;
            return true;
        } else if (!gamepad2.b && b[1]) b[1] = false;
        return false;
    }

}
