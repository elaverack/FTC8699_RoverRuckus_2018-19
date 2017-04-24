package org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions;

import android.content.Context;
import android.content.res.Configuration;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Detects left/right tilt when the phone is in a standing up (portrait) position.
 */
public class TiltDetector implements SensorEventListener {

    private Sensor mRotationVectorSensor;
    private SensorManager mSensorManager;
    private final float[] mRotationMatrix = new float[16];
    float[] result = new float[3];
    private int orientation;


    public TiltDetector(HardwareMap hardwareMap, int orientation) {
        this.orientation = orientation;
        // Get an instance of the SensorManager
        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);

        mRotationVectorSensor = mSensorManager.getDefaultSensor(
                Sensor.TYPE_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);


        DbgLog.msg(mSensorManager.toString());
        DbgLog.msg(mRotationVectorSensor.toString());

        // initialize the rotation matrix to identity
        mRotationMatrix[0] = 1;
        mRotationMatrix[4] = 1;
        mRotationMatrix[8] = 1;
        mRotationMatrix[12] = 1;
    }

    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            // convert the rotation-vector to a 4x4 matrix. the matrix
            // is interpreted by Open GL as the inverse of the
            // rotation-vector, which is what we want.
            SensorManager.getRotationMatrixFromVector(
                    mRotationMatrix, event.values);

            float[] orientationMatrix = new float[3];

            // this remaps the coordinates because the phone is standing up, not laying flat.
            if (orientation == Configuration.ORIENTATION_PORTRAIT) {
                SensorManager
                        .remapCoordinateSystem(mRotationMatrix,
                                SensorManager.AXIS_X, SensorManager.AXIS_Z,
                                mRotationMatrix);


            } else if (orientation == Configuration.ORIENTATION_LANDSCAPE) {
                SensorManager.remapCoordinateSystem(mRotationMatrix, SensorManager.AXIS_Y, SensorManager.AXIS_MINUS_X, mRotationMatrix);
            }
            SensorManager.getOrientation(mRotationMatrix, orientationMatrix);
            result = new float[3];


            result[0] = format(orientationMatrix[0]); //Yaw (azimuth)
            result[1] = format(orientationMatrix[1]); //Pitch
            result[2] = format(orientationMatrix[2]); //Roll

        } else {
            DbgLog.msg("wrong sensorevent: " + event);
        }
    }


    // todo - call this from Autonomous mode to check if robot is tilting
    //
    // todo - you probably want to ignore values under 10 or so. You can use Math.abs(value) to check
    // for that.

    /* Returns last Roll value
     // negative values = tilt left
    // positive values = tilt right
     */

    public float getRoll() {
        if (orientation == Configuration.ORIENTATION_PORTRAIT) {
            return result[2];
        } else {
            return result[2] * -1;
        }
    }

    protected float format(float f) {
        return Math.round(Math.toDegrees((float) f));
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // required by SensorListener interface
    }
}