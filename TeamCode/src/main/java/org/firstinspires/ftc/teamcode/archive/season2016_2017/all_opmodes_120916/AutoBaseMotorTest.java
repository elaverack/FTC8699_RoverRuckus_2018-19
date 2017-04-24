package org.firstinspires.ftc.teamcode.archive.season2016_2017.all_opmodes_120916;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;

public class AutoBaseMotorTest extends AutoBaseOpMode {

    public int getFirstForwardTime() {
        return 2000;
    }



    @Override
    public void runOpMode() throws InterruptedException {

        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);

        waitForStart();

        driveForward(getFirstForwardTime()); // implemented in subclasses
        spinLeft(600);

        driveBackward(2000);

        backUpLeft();
        driveForward(1000);
        backUpRight();


    }
}
