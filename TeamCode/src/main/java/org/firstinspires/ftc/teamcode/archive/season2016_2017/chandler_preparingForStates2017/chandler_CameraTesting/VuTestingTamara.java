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
package org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_CameraTesting;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.ImageAnalyst;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.util.Arrays;

// Created on 2/9/2017 at 6:27 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_CameraTesting

@Autonomous(name = "VuTestingTamara", group = "TEST")  // @Autonomous(...) is the other common choice
@Disabled
public class VuTestingTamara extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private final File PHOTO_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
    private final static int ERROR = 0;
    private final static int BLUE_RED = 1;
    private final static int RED_BLUE = 2;
    private final static int BLUE_BLUE = 3;
    private final static int RED_RED = 4;
    private final int RED_TOLERANCE = 70;
    private final int BLUE_TOLERANCE = 55;

    private int beacon1Config = -1;
    private final int BEACON_1 = 3;
    private int beacon2Config = -1;
    private final int BEACON_2 = 1;
    VuforiaTrackables beacons;
    VuforiaLocalizer vuforia;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        beacons.activate();
        vuforia.setFrameQueueCapacity(5);

        //sleep(2500);

        /*
        vuforia.setFrameQueueCapacity(5);
        Bitmap beacon = null;
        if (beacons.get(3) != null) {
            beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(3).getListener(), vuforia.getCameraCalibration());
        } else {
            DbgLog.msg("Couldn't find beacon...");
        }

        if (beacon != null) {
            savePhoto(beacon, "beacon.png");
        }else {
            DbgLog.msg("Beacon bitmap is null...");
        }
        */

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            checkOnBeacons();

            telemetry.addData("Beacon 1 Config", decodeBeaconConfig(beacon1Config));
            telemetry.addData("Beacon 2 Config", decodeBeaconConfig(beacon2Config));

            telemetry.update();
        }
    }

    private void checkOnBeacons() throws InterruptedException {
        if (beacon1Config <= 0 && beacons.get(BEACON_1) != null) {
            Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(BEACON_1).getListener(), vuforia.getCameraCalibration());
            int[] values = processImageTamara(beacon, RED_TOLERANCE, BLUE_TOLERANCE);
            beacon1Config = processTamaraOutput(values[0], values[1]);
        }
        if (beacon2Config <= 0 && beacons.get(BEACON_2) != null) {
            Bitmap beacon = getBeacon(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), (VuforiaTrackableDefaultListener) beacons.get(BEACON_2).getListener(), vuforia.getCameraCalibration());
            int[] values = processImageTamara(beacon, RED_TOLERANCE, BLUE_TOLERANCE);
            beacon2Config = processTamaraOutput(values[0], values[1]);
        }
    }

    private int processTamaraOutput(int red, int blue) {
        if (red == 1 && blue == 0) {
            return RED_RED;
        } else if (red == 0 && blue == 1) {
            return BLUE_BLUE;
        } else if (red < blue) {
            return RED_BLUE;
        } else if (red > blue) {
            return BLUE_RED;
        } else {
            return ERROR;
        }
    }

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

    // Returns [red, blue, width, height] of bitmap
    private int[] processImageTamara(Bitmap bmp, int redT, int blueT) {

        if (bmp == null) {
            int[] Return = {0, 0, 0, 0};
            return Return;
        }

        int red;
        int blue;
        int width;
        int height;

        bmp = Bitmap.createScaledBitmap(bmp, (int) bmp.getWidth()/16, (int) bmp.getHeight()/16, true);
        width = bmp.getWidth();
        height = bmp.getHeight();
        ImageAnalyst imageAnalyst = new ImageAnalyst(bmp);
        red = imageAnalyst.analyzeDistribution(imageAnalyst.getVerticalDistribution('R', redT));
        blue = imageAnalyst.analyzeDistribution(imageAnalyst.getVerticalDistribution('B', blueT));

        if (red == 0 && blue == 0) {
            switch (imageAnalyst.getDominantColor()) {
                case Color.BLUE:
                    blue = 1;
                    red = 0;
                    break;
                case Color.RED:
                    red = 1;
                    blue = 0;
            }
        }

        int[] Return = {red, blue, width, height};
        return Return;

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
}
