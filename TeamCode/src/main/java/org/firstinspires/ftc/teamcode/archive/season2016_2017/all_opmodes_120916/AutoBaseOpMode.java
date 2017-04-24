package org.firstinspires.ftc.teamcode.archive.season2016_2017.all_opmodes_120916;


import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class AutoBaseOpMode extends LinearOpMode implements SensorEventListener {
    public abstract int getFirstForwardTime();

    protected SensorManager mSensorManager;
    protected Sensor accelerometer;
    protected Sensor magnetometer;

    DcMotor motorRight1;
    DcMotor motorRight2;
    DcMotor motorLeft1;
    DcMotor motorLeft2;

    protected float roll = 0.0f;         // value in radians

    @Override
    public void runOpMode() throws InterruptedException {
        waitOneFullHardwareCycle();
        DbgLog.msg("starting AutoBaseOpMode");
        // DbgLog.msg("hardwareMap.appContext: "+hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE));

        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        DbgLog.msg("sensor manager: " + mSensorManager);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        DbgLog.msg("accelerometer: " + accelerometer);
        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        DbgLog.msg("magnetometer: " + magnetometer);

        mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);

        initMotors();

        waitForStart();
        DbgLog.msg("about to drive forward");
        try {
            driveForward(getFirstForwardTime()); // implemented in subclasses
        } catch (Exception e) {
            DbgLog.error("problem driving: " + e.getMessage());
            DbgLog.logStacktrace(e);
            spinLeft(600); // TODO - change this based on robot tests
            while (opModeIsActive()) {
                if (isTiltingLeft()) {
                    backUpLeft();
                } else if (isTiltingRight()) {
                    backUpRight();
                }
                driveForward(500); // TODO - change this based on robot tests
            }
        }
    }

    // todo - refactor out to a separate class - cut'n'pasted form CompetitionDriver - bad, bad!
    public void initMotors() {

        motorRight1 = hardwareMap.dcMotor.get("right1");
        motorLeft1 = hardwareMap.dcMotor.get("left1");
        motorRight2 = hardwareMap.dcMotor.get("right2");
        motorLeft2 = hardwareMap.dcMotor.get("left2");
        if (motorRight1 == null) {
            DbgLog.error("Missing motor right1");
        }
        if (motorRight2 == null) {
            DbgLog.error("Missing motor right2");
        }
        if (motorLeft1 == null) {
            DbgLog.error("Missing motor left1");
        }
        if (motorLeft2 == null) {
            DbgLog.error("Missing motor left2");
        }
        motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
    }


    public void backUpLeft() throws InterruptedException {
        spinLeft(100);
        driveBackward(200);
        spinRight(100);
    }

    public void backUpRight() throws InterruptedException {
        spinRight(100);
        driveBackward(200);
        spinLeft(100);
    }

    public void driveForward(int milliseconds) throws InterruptedException {
        motorLeft1.setPower(1.0);
        motorRight1.setPower(1.0);


        sleep(milliseconds);

        motorLeft1.setPower(0);
        motorRight1.setPower(0);

    }

    public void driveBackward(int milliseconds) throws InterruptedException {
        motorLeft1.setPower(-1.0);
        motorRight1.setPower(-1.0);

        sleep(milliseconds);

        motorLeft1.setPower(0);
        motorRight1.setPower(0);

    }

    public void spinLeft(int milliseconds) throws InterruptedException {

        motorLeft1.setPower(0.5);
        motorRight1.setPower(-0.5);

        sleep(milliseconds);

        motorLeft1.setPower(0);
        motorRight1.setPower(0);
    }

    public void spinRight(int milliseconds) throws InterruptedException {
        motorLeft1.setPower(-0.5);
        motorRight1.setPower(0.5);

        sleep(milliseconds);

        motorLeft1.setPower(0);
        motorRight1.setPower(0);
    }

    public boolean isTiltingRight() {
        return roll < -25;
    }

    public boolean isTiltingLeft() {
        return roll > 25;
    }

    /* This is just to update roll */
    public void onSensorChanged(SensorEvent event) {
        // we need both sensor values to calculate orientation
        // only one value will have changed when this method called, we assume we can still use the other value.

        float[] mGravity = null;       // latest sensor values
        float[] mGeomagnetic = null;   // latest sensor values
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            mGravity = event.values;
        }

        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            mGeomagnetic = event.values;
        }

        if (mGravity != null && mGeomagnetic != null) {  //make sure we have both before calling getRotationMatrix
            float R[] = new float[9];
            float I[] = new float[9];
            boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
            if (success) {
                float orientation[] = new float[3];
                SensorManager.getOrientation(R, orientation);
                roll = orientation[2] * 57.2957795f; // multiply to convert from degrees to radians
            }
        }
    }


    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // needed to implement SensorListener
    }
}
