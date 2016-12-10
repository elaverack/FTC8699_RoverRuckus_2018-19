package org.firstinspires.ftc.teamcode.barebones_testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.FTDeviceClosedException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.Console;
import java.io.File;

import android.app.Application;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.ContentResolver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.IntentSender;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.database.DatabaseErrorHandler;
import android.database.sqlite.SQLiteDatabase;
import android.graphics.Bitmap;
import android.graphics.drawable.Drawable;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.os.UserHandle;
import android.support.annotation.Nullable;
import android.view.Display;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintWriter;

import java.util.Vector;

// Created on 12/9/2016 at 9:13 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.barebones_testing

@TeleOp(name = "bt_servoMovementRecord", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
//@Disabled
public class bt_servoMovementRecord extends OpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Servo servo;
    TouchSensor sensor;
    Context context;
    File file;
    File file2;
    File file3;

    //Vector save = new Vector();
    //File save;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // Set to REVERSE if using AndyMark motors
        // Set to FORWARD if using AndyMark motors

        servo = hardwareMap.servo.get("servo");
        sensor = hardwareMap.touchSensor.get("sensor");

        //save = new File(context.getFilesDir(), "save.txt");

        context = hardwareMap.appContext;

        //file = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves");
        //if (!file.mkdir()) {
        //    telemetry.addData("Error", "It didn't work, try again...");
        //}

        file = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves", "congrats.txt");
        try {
            file.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("Error", "Maybe not congrats...");
        }

        try {
            FileOutputStream f = new FileOutputStream(file);
            PrintWriter pw = new PrintWriter(f);
            pw.println("YAY! IT WORKED!");
            pw.flush();
            pw.close();
            f.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            telemetry.addData("Error", "Maybe not congrats...");
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("Error", "Maybe not congrats...");
        }

    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {


    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {

        runtime.reset();

        servo.setPosition(0);

    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)

        if (sensor.isPressed()) {
            servo.setPosition(1);
        } else {
            servo.setPosition(0);
        }

        telemetry.addData("Servo Position", servo.getPosition());

        //save.addElement(new Object[]{runtime.toString(), servo.getPosition()});

    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

        // eg: Set all motor powers to 0

        servo.setPosition(0);

    }

}
