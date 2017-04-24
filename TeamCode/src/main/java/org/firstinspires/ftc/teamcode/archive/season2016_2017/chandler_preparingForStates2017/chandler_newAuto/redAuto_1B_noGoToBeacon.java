/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_newAuto;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.MultiplexColorSensor;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.Vector;

// Created on 2/10/2017 at 7:06 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_newAuto

@Autonomous(name = "RED: ONE BEACON (testing)", group = "TEST")
@Disabled
public class redAuto_1B_noGoToBeacon extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private final File PHOTO_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
    private final String SAVE_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves";
    private final static int ERROR = 0;
    private final static int BLUE_RED = 1;
    private final static int RED_BLUE = 2;
    private final static int BLUE_BLUE = 3;
    private final static int RED_RED = 4;
    private final static Scalar BLUE_LOW = new Scalar(100, 0, 220);
    private final static Scalar BLUE_HIGH = new Scalar(178, 255, 255);

    private int beacon1Config = -1;
    private final int BEACON_1 = 3;
    private int beacon2Config = -1;
    private final int BEACON_2 = 1;
    private VuforiaTrackables beacons;
    private VuforiaLocalizer vuforia;

    private MultiplexColorSensor colorSensors;

    final private String MUX_NAME = "mux";
    final private String ADA_NAME = "ada";
    final private int[] PORTS = {0, 1, 2, 3, 4, 5, 6};

    private int[] lastCRGB1 = new int[4];
    private int[] lastCRGB2 = new int[4];
    private int[] crgb1;
    private int[] crgb2;

    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;
    private Servo beacon_presser;
    // Change these after beacon pusher is built and tested
    private final double POSITION_UP = 1;
    private final double POSITION_DOWN = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AfvDu9r/////AAAAGesE+mqXV0hVqVSqU52GJ10v5Scxwd9O/3bf1yzGciRlpe31PP3enyPDvcDbz7KEDxGCONmmpf7+1w7C0PJgkJLNzqxyuHE/pUZlkD37cwnxvJSozZ7I7mx1Vk4Lmw8fAeKlvBAtMCfSeBIPQ89lKkKCuXC7vIjzY66pMmrplByqaq/Ys/TzYkNp8hAwbupsSeykVODtbIbJtgmxeNnSM35zivwcV0hpc5S0oVOoRczJvVxKh5/tzMqH2oQ1fVlNwHhvSnyOGi5L2eoAHyQjsP/96H3vYniltziK13ZmHTM7ncaSC/C0Jt4jL9hHMxvNeFl2Rs7U1u4A+WYJKJ6psFBe2TLJzOwBuzM3KGfZxfkU";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        Vuforia.setFrameFormat(PIXEL_FORMAT.GRAYSCALE, false);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        OpenCVLoader.initDebug();
        beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        colorSensors = new MultiplexColorSensor(hardwareMap, MUX_NAME, ADA_NAME, PORTS, 1, MultiplexColorSensor.GAIN_16X);
        colorSensors.startPolling();

        leftMotor1  = hardwareMap.dcMotor.get("lf");
        leftMotor2  = hardwareMap.dcMotor.get("lb");
        rightMotor1 = hardwareMap.dcMotor.get("rf");
        rightMotor2 = hardwareMap.dcMotor.get("rb");
        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);
        beacon_presser = hardwareMap.servo.get("beacon");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart(); // Wait for the game to start (driver presses PLAY)
        runtime.reset();

        // NOTE: This is basically the heart of the opMode \/
        // It's all functions, but it's the base order for them. The idea was to make it full of functions so it's all very 'plug and play'.
        beacons.activate();
        checkOnBeacons();

        //load(SAVE_DIRECTORY, "RED_MOVE_TO_BEACON_1.txt"); This is a testing class, so there will be no full field to test this movement on...
        telemetry.addData("Status", "Going to white line...");
        telemetry.update();
        goStraightToWhiteLine();
        leftMotor1.setPower(-.1);
        leftMotor2.setPower(-.1);
        rightMotor1.setPower(-.1);
        rightMotor2.setPower(-.1);
        sleep(300);
        telemetry.addData("Status", "Turning...");
        telemetry.update();
        turnToWhiteLine();
        leftMotor1.setPower(.3);
        leftMotor2.setPower(.3);
        rightMotor1.setPower(.3);
        rightMotor2.setPower(.3);
        sleep(2000);
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        sleep(100);
        leftMotor1.setPower(-.3);
        leftMotor2.setPower(-.3);
        rightMotor1.setPower(-.3);
        rightMotor2.setPower(-.3);
        sleep(250);
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

        if (beacon1Config <= 0) {
            checkOnBeacons(1);
        }

        if (beacon1Config > 0) {
            log("Beacon 1 configuration is: " + decodeBeaconConfig(beacon1Config));
            switch (beacon1Config) {
                case BLUE_RED:
                    press(2);
                    break;
                case RED_BLUE:
                    press();
                    break;
                case BLUE_BLUE:
                    press();
                    break;
            }
        } else {
            log("Couldn't recognize beacon for some reason. Trying to press...");
            press();
        }

        // Double check beacon to make sure that the other team isn't getting points
        beacon1Config = -1;
        checkOnBeacons(1);

        if (beacon1Config == BLUE_BLUE) {
            press();
        }

        // TODO: Make a copy and add more here.

        double seconds = runtime.seconds(); // Now that I think about it, this doesn't exactly work... Check out load() code...
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Done.", "Run Time: " + seconds);
            telemetry.update();
        }
    }

    private void press(int times) throws InterruptedException {
        for (int i = 0; i < times && opModeIsActive(); i++) {
            leftMotor1.setPower(-.3);
            leftMotor2.setPower(-.3);
            rightMotor1.setPower(-.3);
            rightMotor2.setPower(-.3);
            sleep(250);
            leftMotor1.setPower(0);
            leftMotor2.setPower(0);
            rightMotor1.setPower(0);
            rightMotor2.setPower(0);
            sleep(100);
            beacon_presser.setPosition(POSITION_DOWN);
            leftMotor1.setPower(.1);
            leftMotor2.setPower(.1);
            rightMotor1.setPower(.1);
            rightMotor2.setPower(.1);
            sleep(1500);
            leftMotor1.setPower(0);
            leftMotor2.setPower(0);
            rightMotor1.setPower(0);
            rightMotor2.setPower(0);
            sleep(100);
            leftMotor1.setPower(-.3);
            leftMotor2.setPower(-.3);
            rightMotor1.setPower(-.3);
            rightMotor2.setPower(-.3);
            sleep(250);
            leftMotor1.setPower(0);
            leftMotor2.setPower(0);
            rightMotor1.setPower(0);
            rightMotor2.setPower(0);
            sleep(100);
        }
        beacon_presser.setPosition(POSITION_UP);
    }

    private void press() throws InterruptedException {
        leftMotor1.setPower(-.3);
        leftMotor2.setPower(-.3);
        rightMotor1.setPower(-.3);
        rightMotor2.setPower(-.3);
        sleep(250);
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        sleep(100);
        beacon_presser.setPosition(POSITION_DOWN);
        leftMotor1.setPower(.1);
        leftMotor2.setPower(.1);
        rightMotor1.setPower(.1);
        rightMotor2.setPower(.1);
        sleep(1500);
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        sleep(100);
        leftMotor1.setPower(-.3);
        leftMotor2.setPower(-.3);
        rightMotor1.setPower(-.3);
        rightMotor2.setPower(-.3);
        sleep(250);
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        beacon_presser.setPosition(POSITION_UP);
    }

    private void log(String message) {
        DbgLog.msg(message);
    }

    // beaconIndex is either 1 or 2, signifying which beacon the robot is in front of.
    private void checkOnBeacons(int beaconIndex) throws InterruptedException {

        if (beaconIndex == 1) {
            if (beacon1Config <= 0 && beacons.get(BEACON_1) != null) {
                vuforia.setFrameQueueCapacity(5);
                Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(BEACON_1).getListener(), vuforia.getCameraCalibration());
                int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
                beacon1Config = values[0];
            } else {
                vuforia.setFrameQueueCapacity(5);
                Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), vuforia.getCameraCalibration());
                int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
                beacon1Config = values[0];
            }
        } else if (beaconIndex == 2) {
            if (beacon2Config <= 0 && beacons.get(BEACON_2) != null) {
                vuforia.setFrameQueueCapacity(5);
                Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(BEACON_2).getListener(), vuforia.getCameraCalibration());
                int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
                beacon2Config = values[0];
            } else {
                vuforia.setFrameQueueCapacity(5);
                Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), vuforia.getCameraCalibration());
                int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
                beacon2Config = values[0];
            }
        }

    }

    private void checkOnBeacons() throws InterruptedException {
        if (beacon1Config <= 0 && beacons.get(BEACON_1) != null) {
            vuforia.setFrameQueueCapacity(5);
            Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(BEACON_1).getListener(), vuforia.getCameraCalibration());
            int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
            beacon1Config = values[0];
        }
        if (beacon2Config <= 0 && beacons.get(BEACON_2) != null) {
            vuforia.setFrameQueueCapacity(5);
            Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(BEACON_2).getListener(), vuforia.getCameraCalibration());
            int[] values = processImageOpenCV(beacon, BLUE_LOW, BLUE_HIGH);
            beacon2Config = values[0];
        }
    }

    // Returns Bitmap's {config, width, height}.
    private int[] processImageOpenCV(Bitmap bmp, Scalar blueLow, Scalar blueHigh) {

        if (bmp == null) {
            int[] Return = {0, 0, 0};
            return Return;
        }

        int config;
        int width;
        int height;

        width = bmp.getWidth();
        height = bmp.getHeight();
        bmp = bmp.copy(Bitmap.Config.RGB_565, true);

        Mat image = new Mat(bmp.getWidth(), bmp.getHeight(), CvType.CV_8UC3);
        Utils.bitmapToMat(bmp, image);

        Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV_FULL);

        Mat mask = new Mat();
        Core.inRange(image, blueLow, blueHigh, mask);
        Moments mmnts = Imgproc.moments(mask, true);

        DbgLog.msg("" + mask.cols() + ", " + mask.rows() + ", " + mask.width() + ", " + mask.height());

        if (mmnts.get_m00() / mask.total() > 0.5) {
            config =  BLUE_BLUE;
            int[] Return = {config, width, height};
            DbgLog.msg("" + (mmnts.get_m00() / mask.total()));
            return Return;
        } else if (mmnts.get_m00() / mask.total() < 0.1) {
            config =  RED_RED;
            int[] Return = {config, width, height};
            return Return;
        } else {
            DbgLog.msg("" + (mmnts.get_m00() / mask.total()));
        }

        if ((mmnts.get_m10()) / (mmnts.get_m00()) <  image.cols() / 2) {
            config =  BLUE_RED;
            DbgLog.msg("" + mmnts.get_m10() + ", " + mmnts.get_m00() + ", " + image.cols());
            int[] Return = {config, width, height};
            return Return;
        } else {
            config =  RED_BLUE;
            int[] Return = {config, width, height};
            return Return;
        }



    }

    // Returns String based on beacon config, e.g. "RED_BLUE".
    private String decodeBeaconConfig(int config) {
        switch (config) {
            case BLUE_RED:
                return "Blue, Red";
            case RED_BLUE:
                return "Red, Blue";
            case BLUE_BLUE:
                return "Blue, Blue";
            case RED_RED:
                return "Red, Red";
            default:
                return "ERROR";
        }
    }

    private Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat) {

        long numImgs = frame.getNumImages();

        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == pixelFormat) {
                return frame.getImage(i);
            } else {
                DbgLog.msg("Not the image I'm looking for. Format: " + frame.getImage(i).getFormat());
            }
        }

        return null;

    }

    private Bitmap getBeacon(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal) {

        OpenGLMatrix pose = beacon.getRawPose();

        if (pose == null) {
            DbgLog.msg("Ahh! Pose is null!");
            return null;
        }

        if (img == null) {
            DbgLog.msg("Ahh! Image is null!");
            return null;
        } else if (img.getPixels() == null) {
            DbgLog.msg("Ahh! Image pixels are null!");
            return null;
        } else {

            //DbgLog.msg("Saving start.png...");
            //savePhoto(img, "start.png");
            //DbgLog.msg("Done. Converting to bitmap...");

            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            float[][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 92, 0)).getData(); //bottom right
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 92, 0)).getData(); //bottom left

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            DbgLog.msg("Done. Saving bitmap.png...");
            savePhoto(bm, "bitmap.png");
            DbgLog.msg("Done. Converting to mat and cropping...");

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, crop);

            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

            Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));
            Core.flip(cropped.t(), cropped, 1);

            DbgLog.msg("Done. Converting back to bitmap and returning...");
            Bitmap Return = Bitmap.createBitmap(cropped.width(), cropped.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(cropped, Return);


            DbgLog.msg("Done getting beacon.");
            return Return;

        }

    }

    private Bitmap getBeacon(Image img, CameraCalibration camCal) {

        if (img == null) {
            DbgLog.msg("Ahh! Image is null!");
            return null;
        } else if (img.getPixels() == null) {
            DbgLog.msg("Ahh! Image pixels are null!");
            return null;
        } else {

            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            DbgLog.msg("Done. Saving bitmap.png...");
            savePhoto(bm, "bitmap.png");
            DbgLog.msg("Done. Converting to mat and rotating...");

            Mat crop = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);
            Utils.bitmapToMat(bm, crop);
            Core.flip(crop.t(), crop, 1);

            DbgLog.msg("Done. Converting back to bitmap and returning...");
            Bitmap Return = Bitmap.createBitmap(crop.width(), crop.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(crop, Return);

            DbgLog.msg("Done getting beacon without cropping.");
            return Return;

        }

    }

    private void savePhoto (Bitmap bmp, String fileName) {

        File file = new File(PHOTO_DIRECTORY, fileName);

        if (!file.exists()) {

            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            bmp.compress(Bitmap.CompressFormat.PNG, 0, bos);
            byte[] bitmapdata = bos.toByteArray();

            try {
                FileOutputStream f = new FileOutputStream(file);
                f.write(bitmapdata);
                f.flush();
                f.close();
            } catch (Exception e) {
                // meh
            }
        }
    }

    private void savePhoto (Mat mat, String fileName) {

        Bitmap bmp = null;
        Utils.bitmapToMat(bmp, mat);

        File file = new File(PHOTO_DIRECTORY, fileName);

        if (!file.exists() && bmp != null) {

            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            bmp.compress(Bitmap.CompressFormat.PNG, 0, bos);
            byte[] bitmapdata = bos.toByteArray();

            try {
                FileOutputStream f = new FileOutputStream(file);
                f.write(bitmapdata);
                f.flush();
                f.close();
            } catch (Exception e) {
                // meh
            }
        }

    }

    private void goStraightToWhiteLine() throws InterruptedException {

        crgb1 = colorSensors.getCRGB(0);
        crgb2 = colorSensors.getCRGB(1);
        lastCRGB1 = crgb1;
        lastCRGB2 = crgb2;

        telemetry.clearAll();

        boolean Start = true;
        boolean Break = false;
        boolean Compare = false;

        // Change the following values accordingly depending on sensor being used and direction
        int[] crgb;
        int[] lastcrgb = lastCRGB1;
        final int PORT = 0;
        final double POWER_1 = .1;
        final double POWER_2 = .1;

        while (opModeIsActive() && !Break) {

            checkOnBeacons();

            if (!Start) {
                crgb = colorSensors.getCRGB(PORT);
                Compare = compareCRGB(lastcrgb, crgb, 850);
            } else {
                crgb = colorSensors.getCRGB(PORT);
                lastcrgb = crgb;
                Start = false;
            }

            /*if (compareCRGB(lastcrgb, crgb) < 2) {
                leftMotor1.setPower(POWER_1);
                leftMotor2.setPower(POWER_1);
                rightMotor1.setPower(POWER_2);
                rightMotor2.setPower(POWER_2);
            } else {
                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }*/

            leftMotor1.setPower(POWER_1);
            leftMotor2.setPower(POWER_1);
            rightMotor1.setPower(POWER_2);
            rightMotor2.setPower(POWER_2);
            telemetry.addData("Compare", Compare);
            telemetry.addData("Last", colorsToString(lastcrgb));
            telemetry.addData("Now", colorsToString(crgb));
            telemetry.update();

            if (!Compare) {
                lastcrgb = crgb;
            }

            Break = Compare;

        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        telemetry.clearAll();

    }

    private void turnToWhiteLine() throws InterruptedException {

        crgb1 = colorSensors.getCRGB(0);
        crgb2 = colorSensors.getCRGB(1);
        lastCRGB1 = crgb1;
        lastCRGB2 = crgb2;

        telemetry.clearAll();

        boolean Start = true;
        boolean Break = false;
        boolean Compare = false;

        // Change the following values accordingly depending on sensor being used and direction
        int[] crgb;
        int[] lastcrgb = lastCRGB2;
        final int PORT = 1;
        final double POWER_1 = -.1;
        final double POWER_2 = .1;

        while (opModeIsActive() && !Break) {

            checkOnBeacons();

            if (!Start) {
                crgb = colorSensors.getCRGB(PORT);
                Compare = compareCRGB(lastcrgb, crgb, 850);
            } else {
                crgb = colorSensors.getCRGB(PORT);
                lastcrgb = crgb;
                Start = false;
            }

            /*if (compareCRGB(lastcrgb, crgb) < 2) {
                leftMotor1.setPower(POWER_1);
                leftMotor2.setPower(POWER_1);
                rightMotor1.setPower(POWER_2);
                rightMotor2.setPower(POWER_2);
            } else {
                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }*/

            leftMotor1.setPower(POWER_1);
            leftMotor2.setPower(POWER_1);
            rightMotor1.setPower(POWER_2);
            rightMotor2.setPower(POWER_2);
            telemetry.addData("Compare", Compare);
            telemetry.addData("Last", colorsToString(lastcrgb));
            telemetry.addData("Now", colorsToString(crgb));
            telemetry.update();

            if (!Compare) {
                lastcrgb = crgb;
            }

            Break = Compare;

        }

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        telemetry.clearAll();

    }

    private boolean compareCRGB(int[] pre, int[] now, int threshold) {
        final int rDif = Math.abs(now[1]-pre[1]);
        final int gDif = Math.abs(now[2]-pre[2]);
        final int bDif = Math.abs(now[3]-pre[3]);
        return rDif+gDif+bDif > threshold;
    }

    private int compareCRGB(int[] pre, int[] now) {
        final int rDif = Math.abs(now[1]-pre[1]);
        final int gDif = Math.abs(now[2]-pre[2]);
        final int bDif = Math.abs(now[3]-pre[3]);
        return rDif+gDif+bDif;
    }

    private String colorsToString (int[] colors) {
        String Return = "";
        for (int i = 0; i < colors.length; i++) {
            if (i < colors.length-1) {
                Return += colors[i] + ", ";
            } else {
                Return += colors[i];
            }
        }
        return Return;
    }

    private void load (String path, String fileName) throws InterruptedException {
        double[][] values;
        File loadFile;
        int counter = 0;
        double LF = 0;
        double LB = 0;
        double RF = 0;
        double RB = 0;

        loadFile= new File(path, fileName);
        if (!loadFile.exists()) {
            //Double check if the file exists in order to avoid errors later.
            telemetry.addData("Error", "Couldn't find load file");
        }



        Vector load = new Vector();
        String[] e;
        try {
            FileInputStream f = new FileInputStream(loadFile);
            BufferedReader reader = new BufferedReader(new InputStreamReader(f));

            String line = reader.readLine();
            while(line != null){
                load.addElement(line);
                line = reader.readLine();
            }

        } catch (FileNotFoundException er) {
            // meh
        } catch (IOException er) {
        } finally {
            e = Arrays.copyOf(load.toArray(), load.toArray().length, String[].class);
        }

        values = new double[e.length-2][5];

        for (int i = 1; i < e.length-1; i++) {
            //double TIME = Double.parseDouble(e[i].substring(e[i].indexOf('{')+1, e[i].indexOf('}')));
            //double POWER = Double.parseDouble(e[i].substring(e[i].lastIndexOf('{')+1, e[i].lastIndexOf('}')));
            double[] timeAndPowers = processString(e[i]);

            values[i-1] = timeAndPowers;
        }

        runtime.reset();

        while (counter < values.length) {
            if (runtime.seconds() > values[counter][0]) {
                //servo.setPosition(values[counter][1]);
                LF = values[counter][1];
                LB = values[counter][2];
                RF = values[counter][3];
                RB = values[counter][4];
                counter++;
            }

            leftMotor1.setPower(LF);
            leftMotor2.setPower(LB);
            rightMotor1.setPower(RF);
            rightMotor2.setPower(RB);

            checkOnBeacons();

            telemetry.addData("Values Length", values.length);
            telemetry.addData("Counter", counter);
            telemetry.addData("Power", "Right Front: " + RF);
            telemetry.addData("Power", "Right Back: " + RB);
            telemetry.addData("Power", "Left Front: " + LF);
            telemetry.addData("Power", "Left Back: " + LB);

        }

        telemetry.clearAll();

    }

    private double[] processString(String string) {
        double TIME = Double.parseDouble(string.substring(string.indexOf('{')+1, string.indexOf('}')));
        String newString = string.substring(string.indexOf('}')+1);
        double LF = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        double LB = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        double RF = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        double RB = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        return new double[]{TIME, LF, LB, RF, RB};

    }

}
