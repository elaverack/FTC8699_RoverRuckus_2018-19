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
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.ImageAnalyst;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;


// Created on 2/3/2017 at 9:59 AM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes

@TeleOp(name = "PhotoTest3", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class BETA_Photo3 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private final static Scalar BLUE_LOW = new Scalar(100, 0, 220);
    private final static Scalar BLUE_HIGH = new Scalar(178, 255, 255);

    private final static int ERROR = 0;
    private final static int BLUE_RED = 1;
    private final static int RED_BLUE = 2;
    private final static int BLUE_BLUE = 3;
    private final static int RED_RED = 4;

    private final File PHOTO_DIRECTORY = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
    private final int RED_TOLERANCE = 70;
    private final int BLUE_TOLERANCE = 55;
    private final int[] photoNames = {1, 2, 3, 4};
    // First value for photo number, second looks like this: red, blue, width, height
    private int[][] valuesTamara = new int[4][4];
    // First value for photo number, second looks like this: config, width, height
    private int[][] valuesOpenCV = new int[4][3];


    @Override
    public void runOpMode() throws InterruptedException {

        OpenCVLoader.initDebug();

        telemetry.addData("Status", "Initialized.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double processTime1;
        double processTime2;
        runtime.reset();
        for (int i = 0; i < photoNames.length; i++) {

            File photoFile = new File(PHOTO_DIRECTORY, photoNames[i] + ".jpg");
            Bitmap photo = BitmapFactory.decodeFile(photoFile.getPath());
            valuesTamara[i] = processImageTamara(photo, RED_TOLERANCE, BLUE_TOLERANCE);

        }
        processTime1 = runtime.seconds();
        runtime.reset();
        for (int i = 0; i < photoNames.length; i++) {

            File photoFile = new File(PHOTO_DIRECTORY, photoNames[i] + ".jpg");
            Bitmap photo = BitmapFactory.decodeFile(photoFile.getPath());
            valuesOpenCV[i] = processImageOpenCV(photo, BLUE_LOW, BLUE_HIGH);

        }
        processTime2 = runtime.seconds();

        /*String photo3 = "";
        String photo4 = "";
        if (true) {
            File photoFile = new File(PHOTO_DIRECTORY, "3.jpg");
            Bitmap photo = BitmapFactory.decodeFile(photoFile.getPath());
            photo3 = colorToString(ImageAnalyst.getDominantColor(photo));
            photoFile = new File(PHOTO_DIRECTORY, "3.jpg");
            photo = BitmapFactory.decodeFile(photoFile.getPath());
            photo4 = colorToString(ImageAnalyst.getDominantColor(photo));
        }*/

        // run until the end of the match (driver presses STOP)
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Guidelines", "The values should be:");
            telemetry.addData("Photo 1", "BLUE, RED");
            telemetry.addData("Photo 2", "RED, BLUE");
            telemetry.addData("Photo 3", "RED, RED");
            telemetry.addData("Photo 4", "BLUE, BLUE");

            telemetry.addData("Tamara Process", "Time: " + processTime1 + " seconds.");
            telemetry.addData("TPhoto 1", String.format("RED: %1$s, BLUE: %2$s, W: %3$s, H: %4$s", valuesTamara[0][0], valuesTamara[0][1], valuesTamara[0][2], valuesTamara[0][3]));
            telemetry.addData("TPhoto 2", String.format("RED: %1$s, BLUE: %2$s, W: %3$s, H: %4$s", valuesTamara[1][0], valuesTamara[1][1], valuesTamara[1][2], valuesTamara[1][3]));
            telemetry.addData("TPhoto 3", String.format("RED: %1$s, BLUE: %2$s, W: %3$s, H: %4$s", valuesTamara[2][0], valuesTamara[2][1], valuesTamara[2][2], valuesTamara[2][3]));
            telemetry.addData("TPhoto 4", String.format("RED: %1$s, BLUE: %2$s, W: %3$s, H: %4$s", valuesTamara[3][0], valuesTamara[3][1], valuesTamara[3][2], valuesTamara[3][3]));
            telemetry.addData("TPhoto1C", decodeBeaconConfig(processTamaraOutput(valuesTamara[0][0], valuesTamara[0][1])));
            telemetry.addData("TPhoto2C", decodeBeaconConfig(processTamaraOutput(valuesTamara[1][0], valuesTamara[1][1])));
            telemetry.addData("TPhoto3C", decodeBeaconConfig(processTamaraOutput(valuesTamara[2][0], valuesTamara[2][1])));
            telemetry.addData("TPhoto4C", decodeBeaconConfig(processTamaraOutput(valuesTamara[3][0], valuesTamara[3][1])));

            telemetry.addData("openCV", "Time: " + processTime2 + " seconds.");
            telemetry.addData("CPhoto 1", String.format("Config: %1$s, W: %2$s, H: %3$s", decodeBeaconConfig(valuesOpenCV[0][0]), valuesOpenCV[0][1], valuesOpenCV[0][2]));
            telemetry.addData("CPhoto 2", String.format("Config: %1$s, W: %2$s, H: %3$s", decodeBeaconConfig(valuesOpenCV[1][0]), valuesOpenCV[1][1], valuesOpenCV[1][2]));
            telemetry.addData("CPhoto 3", String.format("Config: %1$s, W: %2$s, H: %3$s", decodeBeaconConfig(valuesOpenCV[2][0]), valuesOpenCV[2][1], valuesOpenCV[2][2]));
            telemetry.addData("CPhoto 4", String.format("Config: %1$s, W: %2$s, H: %3$s", decodeBeaconConfig(valuesOpenCV[3][0]), valuesOpenCV[3][1], valuesOpenCV[3][2]));

            //telemetry.addData("Photo 3 'Dominant Color'", photo3);
            //telemetry.addData("Photo 4 'Dominant Color'", photo4);

            telemetry.update();
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

    // Returns [red, blue, width, height] of bitmap
    private int[] processImageTamara(Bitmap bmp, int redT, int blueT) {
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

    // Returns [config, width, height] of bitmap
    private int[] processImageOpenCV(Bitmap bmp, Scalar blueLow, Scalar blueHigh) {
        int config;
        int width;
        int height;

        width = bmp.getWidth();
        height = bmp.getHeight();
        bmp = bmp.copy(Bitmap.Config.RGB_565, true);

        Mat image = new Mat(bmp.getWidth(), bmp.getHeight(), CvType.CV_8UC3);
        Utils.bitmapToMat(bmp, image);
        //Core.flip(image.t(), image, 1);

        //Bitmap flipped = Bitmap.createBitmap(image.width(), image.height(), Bitmap.Config.RGB_565);
        //Utils.matToBitmap(image, flipped);

        Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2HSV_FULL);

        Mat mask = new Mat();
        Core.inRange(image, blueLow, blueHigh, mask);
        Moments mmnts = Imgproc.moments(mask, true);

        /*Bitmap maskBit = Bitmap.createBitmap(mask.width(), mask.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(mask, maskBit);

        File flippedFile = new File(PHOTO_DIRECTORY, "flip.png");

        if (!flippedFile.exists()) {

            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            flipped.compress(Bitmap.CompressFormat.PNG, 0, bos);
            byte[] bitmapdata = bos.toByteArray();

            try {
                FileOutputStream f = new FileOutputStream(flippedFile);
                f.write(bitmapdata);
                f.flush();
                f.close();
            } catch (Exception e) {
                // meh
            }

        }*/

        //Core.flip(mask.t(), mask, 1);
        DbgLog.msg("" + mask.cols() + ", " + mask.rows() + ", " + mask.width() + ", " + mask.height());

        /*File maskFile = new File(PHOTO_DIRECTORY, "mask.png");

        if (!maskFile.exists()) {

            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            maskBit.compress(Bitmap.CompressFormat.PNG, 0 , bos);
            byte[] bitmapdata = bos.toByteArray();

            try {
                FileOutputStream f = new FileOutputStream(maskFile);
                f.write(bitmapdata);
                f.flush();
                f.close();
            } catch (Exception e) {
                // meh
            }

        }*/

        if (mmnts.get_m00() / mask.total() > 0.2) {
            config =  BLUE_BLUE;
            int[] Return = {config, width, height};
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

    private String colorToString (int color) {
        switch (color) {
            case Color.BLACK:
                return "BLACK";
            case Color.BLUE:
                return "BLUE";
            case Color.CYAN:
                return "CYAN";
            case Color.DKGRAY:
                return "DARK GREY";
            case Color.GRAY:
                return "GREY";
            case Color.GREEN:
                return "GREEN";
            case Color.LTGRAY:
                return "LIGHT GREY";
            case Color.MAGENTA:
                return "MAGENTA";
            case Color.RED:
                return "RED";
            case Color.TRANSPARENT:
                return "TRANSPARENT";
            case Color.WHITE:
                return "WHITE";
            case Color.YELLOW:
                return "YELLOW";
            default:
                return "ERROR";
        }
    }

}
