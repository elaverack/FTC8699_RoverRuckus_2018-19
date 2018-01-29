package org.firstinspires.ftc.teamcode.unnamed;

// Created on 1/20/2018 at 12:59 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class unnamed_v2 {

    protected static final int thres = 10;

    protected static final String
            drive_rfN = "rf",   // The front right motor name
            drive_rbN = "rb",   // The back right motor name
            drive_lfN = "lf",   // The front left motor name
            drive_lbN = "lb",   // The back left motor name

            // Note: Looking at robot, not from
            grabber_blN = "bl", // The bottom left grabber servo name
            grabber_brN = "br", // The bottom right grabber servo name
            grabber_tlN = "tl", // The top left grabber servo name
            grabber_trN = "tr", // The top right grabber servo name

            liftN = "lift",     // The lift motor name

            jewelN = "jewel",   // The jewel arm servo motor name
            colorN = "color",   // The jewel arm color sensor name
            gyroN = "gyro";     // The gyro sensor name

    private static final double
            jewel_down = .18,   // The jewel servo position for lowering the arm
            jewel_up = 1;       // The jewel servo position for raising the arm

    protected OpMode opmode;
    protected DcMotor rf, rb, lf, lb;
    protected G_Grabber g;
    protected Lift l;

    public ModernRoboticsI2cColorSensor c;
    private Servo j;
    public GyroSensor gyro;

    public unnamed_v2(OpMode om) {
        opmode = om;

        rf = opmode.hardwareMap.dcMotor.get(drive_rfN);
        rb = opmode.hardwareMap.dcMotor.get(drive_rbN);
        lf = opmode.hardwareMap.dcMotor.get(drive_lfN);
        lb = opmode.hardwareMap.dcMotor.get(drive_lbN);

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);

        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void auto_init() {
        c = (ModernRoboticsI2cColorSensor) opmode.hardwareMap.colorSensor.get(colorN);
        j = opmode.hardwareMap.servo.get(jewelN);
        gyro = opmode.hardwareMap.gyroSensor.get(gyroN);
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            opmode.telemetry.addData("Status","Waiting for gyro...");
            opmode.telemetry.update();
        }
    }

    public void j_down () { j.setPosition(jewel_down); }
    public void j_up () { j.setPosition(jewel_up); }

    public void drive_init() {
        g = new G_Grabber();
        l = new Lift();
        g.init();
        l.init();
    }

    public void start () { g.start(); l.start(); opmode.hardwareMap.servo.get(jewelN).setPosition(jewel_up); }

    public void drive () {

        float straight = -opmode.gamepad1.right_stick_y;
        float strafe = opmode.gamepad1.right_stick_x;
        float rotate = opmode.gamepad1.left_stick_x;
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

        if (opmode.gamepad1.right_trigger > 0.5) { powerLB /= 4; powerLF /= 4; powerRB /= 4; powerRF /= 4; }

        lf.setPower(powerLF);
        lb.setPower(powerLB);
        rf.setPower(powerRF);
        rb.setPower(powerRB);

        g.run(opmode.gamepad1.a, opmode.gamepad1.b);
        l.run(opmode.gamepad1.dpad_up, opmode.gamepad1.dpad_down, opmode.gamepad1.y, opmode.gamepad1.x);

    }

    public void sensorOut() {
        opmode.telemetry.addData(colorN, c.argb());
        opmode.telemetry.addData(gyroN, gyro.rawX() + ", " + gyro.rawY() + ", " + gyro.rawZ() + "; h: " + gyro.getHeading());
    }

    private boolean rbumbd = false;
    public void drive_jewel () {
        if (!rbumbd && opmode.gamepad1.right_bumper) {
            if (j.getPosition() == jewel_down) {
                j.setPosition(jewel_up);
            } else j.setPosition(jewel_down);
            rbumbd = true;
        } else if (!opmode.gamepad1.right_bumper) rbumbd = false;
    }

    public void stop () {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        l.stop();
    }

    public void stop_drive () {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
    }

    public void drive_distance (double inchesdistance) {

        double C = Math.PI * 4.000;
        int goal = (int)((inchesdistance / C)*(1120));



    }

    private class G_Grabber {

        private G_servo bl, br, tl, tr;

        // TODO: Add in safe guards in case grabber servos loosen again
        private static final double // Servo positions
                blo = 0.078,    // Bottom left open
                blc = 0.310,    // Bottom left close
                bro = 0.98,     // Bottom right open
                brc = 0.670,    // Bottom right close
                tlo = 0.98,     // Top left open
                tlc = 0.725,    // Top left close
                tro = 0.118,    // Top right open
                trc = 0.392;    // Top right close

        void init () {
            bl = new G_servo(grabber_blN, blo, blc);
            br = new G_servo(grabber_brN, bro, brc);
            tl = new G_servo(grabber_tlN, tlo, tlc);
            tr = new G_servo(grabber_trN, tro, trc);
        }

        void start () { open_b(); open_t(); }

        // TODO: Add code for rotation
        private boolean b_togd = false, t_togd = false;
        void run (boolean b_tog, boolean t_tog) {
            if (b_tog && !b_togd) {
                tog_b();
                b_togd = true;
            } else if (!b_tog) b_togd = false;
            if (t_tog && !t_togd) {
                tog_t();
                t_togd = true;
            } else if (!t_tog) t_togd = false;
        }

        void open_b  () { bl.open();    br.open();      }
        void open_t  () { tl.open();    tr.open();      }
        void close_b () { bl.close();   br.close();     }
        void close_t () { tl.close();   tr.close();     }
        void tog_b   () { bl.toggle();  br.toggle();    }
        void tog_t   () { tl.toggle();  tr.toggle();    }

        private class G_servo {

            private double c, o;
            private Servo s;

            G_servo (String config_name, double open_pos, double close_pos) {
                s = opmode.hardwareMap.servo.get(config_name);
                o = open_pos; c = close_pos;
            }

            void close () { s.setPosition(c); }
            void open () { s.setPosition(o); }
            void toggle () { if (s.getPosition() == o) { close(); } else open(); }

        }

    }

    protected class Lift {

        private DcMotor l;

        private static final int
                lift0 = 0,
                lift1 = 3000,
                lift2 = 5330;

        private static final double
                liftS   = .55,
                liftDS  = .25;

        void init () {
            l = opmode.hardwareMap.dcMotor.get(liftN);
            l.setDirection(DcMotorSimple.Direction.REVERSE);
            l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        void start () {
            if (lift0 == 0) return;
            l.setTargetPosition(lift0);
            l.setPower(liftS);
            while (!update_encoders(l));
        }

        // TODO: Add grounding code (and method)
        private boolean upd = false, gd = false, ddupd = false, dddownd = false;
        void run (boolean up, boolean ground, boolean dd_up, boolean dd_down) {

            if (!upd && up) {
                switch (l.getTargetPosition()) {
                    case lift0:
                        l.setTargetPosition(lift1); l.setPower(liftS);
                        break;
                    case lift1:
                        l.setTargetPosition(lift2); l.setPower(liftS);
                        break;
                    case lift2:
                        l.setTargetPosition(lift1); l.setPower(-liftS);
                        break;
                }
                upd = true;
            } else if (!up) upd = false;

            if (!gd && ground) { l.setTargetPosition(lift0); l.setPower(-liftS); gd = true; }
            else if (!ground) gd = false;

            update_encoders(l);

            if (!ddupd && dd_up) {
                l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                l.setPower(liftDS);
                ddupd = true;
            } else if (ddupd && !dd_up) {
                l.setPower(0);
                ddupd = false;
                l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (!dddownd && dd_down) {
                l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                l.setPower(-liftDS);
                dddownd = true;
            } else if (dddownd && !dd_down) {
                l.setPower(0);
                dddownd = false;
                l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

        }

        void stop() { l.setPower(0); }

    }

    private boolean update_encoders (DcMotor m) {
        if (m.getMode() != DcMotor.RunMode.RUN_TO_POSITION) return false;
        boolean ret = m.getCurrentPosition() < m.getTargetPosition() + thres &&
                m.getCurrentPosition() > m.getTargetPosition() - thres;
        if (ret) m.setPower(0);
        return ret;
    }

}
