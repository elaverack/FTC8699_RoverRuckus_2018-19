package org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_newAuto.tele_records;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.Vector;

// Created on 1/18/2017 at 4:42 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.archive.season2016_2017.chandler_preparingForStates2017.chandler_betaOpmodes

@TeleOp(name = "Rec: BLUE2 SHOOT", group = "REC")
@Disabled
public class BLUE2_GO_TO_SHOOTING_POSITION extends OpMode {
    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Copy this class and rename this
    private final String FILE_NAME = "BLUE2_GO_TO_SHOOTING_POSITION.txt";

    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    private float rightStickY;
    private float rightStickX;
    private float powerRF;
    private float powerRB;
    private float powerLF;
    private float powerLB;
    private float leftStickX;
    private float[] currentValues = new float[4];
    private float[] previousValues = new float[]{0, 0, 0, 0};

    private Vector save = new Vector();
    private File saveFile;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor1  = hardwareMap.dcMotor.get("lf");
        leftMotor2  = hardwareMap.dcMotor.get("lb");
        rightMotor1 = hardwareMap.dcMotor.get("rf");
        rightMotor2 = hardwareMap.dcMotor.get("rb");

        leftMotor1.setDirection(DcMotor.Direction.FORWARD);
        leftMotor2.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);

        saveFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/Servo Test Saves", FILE_NAME);
        if (saveFile.exists()) {
            saveFile.delete();
        }
        try {
            saveFile.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("Error", "Couldn't create file for some reason.");
        }

    }

    @Override
    public void init_loop() {


    }

    @Override
    public void start() {

        runtime.reset();

        save.addElement("START");
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        rightStickY = -gamepad1.right_stick_y;
        rightStickX = gamepad1.right_stick_x;
        leftStickX = gamepad1.left_stick_x;
        powerRF = rightStickY;
        powerRB = rightStickY;
        powerLF = rightStickY;
        powerLB = rightStickY;
        powerRF -= rightStickX;
        powerRB += rightStickX;
        powerLF += rightStickX;
        powerLB -= rightStickX;
        powerRF -= leftStickX;
        powerRB -= leftStickX;
        powerLF += leftStickX;
        powerLB += leftStickX;

        if (powerRF > 1) {powerRF = 1;} else if (powerRF < -1) {powerRF = -1;}
        if (powerRB > 1) {powerRB = 1;} else if (powerRB < -1) {powerRB = -1;}
        if (powerLF > 1) {powerLF = 1;} else if (powerLF < -1) {powerLF = -1;}
        if (powerLB > 1) {powerLB = 1;} else if (powerLB < -1) {powerLB = -1;}

        leftMotor1.setPower(powerLF);
        leftMotor2.setPower(powerLB);
        rightMotor1.setPower(powerRF);
        rightMotor2.setPower(powerRB);

        currentValues = new float[]{powerLF, powerLB, powerRF, powerRB};

        if (!compareValues(previousValues, currentValues)) {
            save.addElement("{" + runtime.seconds() + "}, {" + powerLF + "}, {" + powerLB + "}, {" + powerRF + "}, {" + powerRB + "}");
        }

        previousValues = currentValues;
    }

    @Override
    public void stop() {

        save.addElement("STOP");

        try {
            FileOutputStream f = new FileOutputStream(saveFile);
            PrintWriter pw = new PrintWriter(f);
            Enumeration e = save.elements();

            while (e.hasMoreElements()) {
                pw.println(e.nextElement().toString());
            }
            pw.flush();
            pw.close();
            f.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            telemetry.addData("Error", "I wonder if this shows up");
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addData("Error", "I wonder if this shows up");
        }

    }

    private boolean compareValues(float[] prior, float[] present) {
        return Arrays.equals(prior, present);
    }

}
