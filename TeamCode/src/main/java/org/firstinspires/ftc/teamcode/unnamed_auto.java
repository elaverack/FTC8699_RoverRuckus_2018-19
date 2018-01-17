package org.firstinspires.ftc.teamcode;

// Created on 1/13/2018 at 11:47 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class unnamed_auto extends unnamed {

    protected static final String // TODO: Write in sensor config names
            cs_rightleftN = "color",
            cs_middleN = "",
            cs_jewelN = "",
            us_wall1N = "ultra",
            us_wall2N = "",
            gyroN = "gyro";

    protected Line_cs
            rls,    // sensor for right/left column alignment
            ms,     // sensor for middle column alignment
            jewels; // sensor for detecting jewels (backup for phone camera)
    protected Wall_us
            wus1,   // sensor for detecting distance to wall (for aligning to columns)
            wus2;   // sensor (backup) for detecting distance to wall TODO: Add code for us2 if need be
    protected Gyro_sensor gyro;

    // TODO: Add visuals handler along with init and utility functions as need be

    public unnamed_auto (OpMode om) { opmode = om; super.init(); this.init(); }

    protected void init () { // TODO: Add jewel sensor init code if needed
        rls = new Line_cs(cs_rightleftN);
        //ms = new Line_cs(cs_middleN); TODO: name (in configuration) and enable middle sensor

        wus1 = new Wall_us(us_wall1N);

        gyro = new Gyro_sensor(gyroN);
        gyro.calibate(); gyro.tellDriverWaiting(); // TODO: Change so that while waiting for calibration, calculate pos info
    }

    public void start () {
        super.start();
        rls.start();
        //ms.start(); TODO: enable middle sensor
    }

    class Line_cs { // Line color sensor (we are going to be using MR color sensors)

        private ModernRoboticsI2cColorSensor s;
        private String cn;

        Line_cs (String config_name) {
            cn = config_name;
            s = (ModernRoboticsI2cColorSensor) opmode.hardwareMap.colorSensor.get(cn);
        }

        public void start () { s.enableLight(true); s.enableLed(true); }

        public void tele_out () { opmode.telemetry.addData(cn, s.argb()); opmode.telemetry.update(); }

        public boolean over_red_line () {return false;} // TODO: write over red line code

        public boolean over_blue_line () {return false;} // TODO: write over blue line code

    }

    class Wall_us { // TODO: write utility methods for ultrasonic sensor for wall

        private UltrasonicSensor s;
        private String cn;

        Wall_us (String config_name) { cn = config_name; s = opmode.hardwareMap.ultrasonicSensor.get(cn); }

        public void tele_out () { opmode.telemetry.addData(cn, s.getUltrasonicLevel()); opmode.telemetry.update(); }

    }

    class Gyro_sensor { // TODO: write utility methods for gyro sensor

        private GyroSensor s;
        private String cn;
        public boolean calibrated = false; // WARNING: only checks if gyro is calibrated, not if its done calibrating

        Gyro_sensor (String config_name) { cn = config_name; s = opmode.hardwareMap.gyroSensor.get(cn); }

        public void calibate () { s.calibrate(); calibrated = true; }

        public void waitForCalibrate () { while (s.isCalibrating()); }

        public void tellDriverWaiting () {
            while (s.isCalibrating()) {
                opmode.telemetry.addData("Status", "Probably shouldn't move, gyro is calibrating...");
                opmode.telemetry.update();
            }
        }

        public boolean isCalibrating () { return s.isCalibrating(); }

        public void tele_out () {
            opmode.telemetry.addData(cn, s.rawX() + ", " + s.rawY() + ", " + s.rawZ() + "; h: " + s.getHeading());
            opmode.telemetry.update();
        }

    }
}
