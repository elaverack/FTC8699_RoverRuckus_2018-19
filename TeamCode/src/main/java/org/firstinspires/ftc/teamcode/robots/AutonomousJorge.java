package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotHandlers.EncodedRobotDrive;
import org.firstinspires.ftc.teamcode.robotHandlers.MultiplexedColorSensors;
import org.firstinspires.ftc.teamcode.robotHandlers.OptimizedGamepadRecorder;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotConfig;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotEncodedMotors;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotHandler;
import org.firstinspires.ftc.teamcode.robotHandlers.RobotServos;
import org.firstinspires.ftc.teamcode.robotHandlers.StandardRobotDrive;
import org.firstinspires.ftc.teamcode.robotHandlers.Vuforia2017Manager;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.Vector;

/**
 * Created by Chandler on 4/4/2017.
 */

public class AutonomousJorge extends Jorge {

    public MultiplexedColorSensors colorSensors;
    public LinearOpMode opMode;
    public Vuforia2017Manager vuforia;
    public static final String
            BEACON_PRESSER_L        = "beaconl",
            BEACON_PRESSER_R        = "beaconr";

    private final int
            NUMBER_OF_SENSORS       = 2;

    public static final double
            BEACON_PRESSER_R_UP     = 0,
            BEACON_PRESSER_R_DOWN   = 1;

    public enum COLOR_SENSOR {
        MIDDLE(0), BACK(1);

        public final int port;

        COLOR_SENSOR (int port) {this.port = port;}
    }

    public enum BEACON {
        B1(1), B2(2);

        public final int index;

        BEACON (int index) {this.index = index;}
    }

    public AutonomousJorge(LinearOpMode om) { init(om, true); }

    public AutonomousJorge(LinearOpMode om, boolean doVuforia) { init(om, doVuforia); }

    public void playRecordingRed (String fileName ) throws Error {

        double[][] values;
        File loadFile;
        int counter = 0;
        double
                LF = 0,
                LB = 0,
                RF = 0,
                RB = 0;

        Vector recording = new Vector();
        String[] e;

        ElapsedTime runtime = new ElapsedTime();

        loadFile= new File(OptimizedGamepadRecorder.RECORD_DIR, fileName);
        if (!loadFile.exists()) {
            //Double check if the file exists in order to avoid errors later.
            throw new Error("Couldn't find load file");
        }



        try {
            FileInputStream f = new FileInputStream(loadFile);
            BufferedReader reader = new BufferedReader(new InputStreamReader(f));

            String line = reader.readLine();
            while(line != null){
                recording.addElement(line);
                line = reader.readLine();
            }

        } catch (FileNotFoundException er) {
            throw new Error("Couldn't find load file");
        } catch (IOException er) {
            throw new Error("Error: IOException...");
        } finally {
            e = Arrays.copyOf(recording.toArray(), recording.toArray().length, String[].class);
        }

        values = new double[e.length-2][5];

        for (int i = 1; i < e.length-1; i++) {
            //double TIME = Double.parseDouble(e[i].substring(e[i].indexOf('{')+1, e[i].indexOf('}')));
            //double POWER = Double.parseDouble(e[i].substring(e[i].lastIndexOf('{')+1, e[i].lastIndexOf('}')));
            double[] timeAndPowers = processString(e[i]);

            values[i-1] = timeAndPowers;
        }

        opMode.telemetry.clearAll();
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

            /*leftMotor1.setPower(LF);
            leftMotor2.setPower(LB);
            rightMotor1.setPower(RF);
            rightMotor2.setPower(RB);*/
            drive.setPowers( new String[]{"lf", "lb", "rf", "rb"}, new double[]{LF, LB, RF, RB} );

            opMode.telemetry.addData("Values Length", values.length);
            opMode.telemetry.addData("Counter", counter);
            opMode.telemetry.addData("Power", "Right Front: " + RF);
            opMode.telemetry.addData("Power", "Right Back: " + RB);
            opMode.telemetry.addData("Power", "Left Front: " + LF);
            opMode.telemetry.addData("Power", "Left Back: " + LB);

        }

        opMode.telemetry.clearAll();

    }

    private double[] processString(String string) {
        double TIME = Double.parseDouble(string.substring(string.indexOf('{')+1, string.indexOf('}')));
        String newString = string.substring(string.indexOf('}')+1);
        double straight = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        double rotate = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        double strafe = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));
        newString = newString.substring(newString.indexOf('}')+1);
        double slow = Double.parseDouble(newString.substring(newString.indexOf('{')+1, newString.indexOf('}')));

        double RF = straight;
        double RB = straight;
        double LF = straight;
        double LB = straight;
        RF -= strafe;
        RB += strafe;
        LF += strafe;
        LB -= strafe;
        RF -= rotate;
        RB -= rotate;
        LF += rotate;
        LB += rotate;

        if (RF > 1) {RF = 1;} else if (RF < -1) {RF = -1;}
        if (RB > 1) {RB = 1;} else if (RB < -1) {RB = -1;}
        if (LF > 1) {LF = 1;} else if (LF < -1) {LF = -1;}
        if (LB > 1) {LB = 1;} else if (LB < -1) {LB = -1;}

        if (slow > 0.25) {
            RF /= 4 * slow;
            RB /= 4 * slow;
            LF /= 4 * slow;
            LB /= 4 * slow;
        }

        return new double[]{TIME, LF, LB, RF, RB};

    }

    private void init(LinearOpMode om, boolean doVuforia) {
        this.opMode = om;
        this.config = new RobotConfig(4, RobotConfig.driveType.MECANUM, 2, 3);
        drive = new EncodedRobotDrive(opMode.hardwareMap);
        drive.setSideDirections(
                new StandardRobotDrive.SIDE[]{StandardRobotDrive.SIDE.RIGHT, StandardRobotDrive.SIDE.LEFT},
                new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD}
        );
        auxMotors = new RobotEncodedMotors(opMode.hardwareMap, new String[]{SHOOTER, PICKUP});
        servos = new RobotServos(opMode.hardwareMap, new String[]{BEACON_PRESSER_L, BEACON_PRESSER_R, LOADER});

        colorSensors = new MultiplexedColorSensors(opMode.hardwareMap, "mux", "ada", NUMBER_OF_SENSORS, MultiplexedColorSensors.ATIME.FASTEST, MultiplexedColorSensors.GAIN._16X);
        vuforia = doVuforia ? new Vuforia2017Manager() : null;
    }

    public void shoot ( int noTimes ) {
        if (!opMode.opModeIsActive() || noTimes < 1 || noTimes > 2) return;
        auxMotors.setMode(SHOOTER, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        auxMotors.setMode(SHOOTER, DcMotor.RunMode.RUN_TO_POSITION);
        int goalEncoder = 1680*noTimes;
        boolean loadedSecond = false;
        servos.setPosition(LOADER, LOAD);
        auxMotors.setTargetPosition(SHOOTER, goalEncoder, .5);
        while (/*auxMotors.getPosition(SHOOTER) > SHOOTER_FIRE || auxMotors.getPosition(SHOOTER) == SHOOTER_FIRE*/
                !compareTarget(new int[]{auxMotors.getPosition(SHOOTER)}, new int[]{goalEncoder})) {
            if (!opMode.opModeIsActive()) return;
            if (noTimes > 1 && auxMotors.getPosition(SHOOTER) > goalEncoder/2 && !loadedSecond) {
                auxMotors.setPower(SHOOTER, 0);
                servos.setPosition(LOADER, UNLOAD);
                ElapsedTime runtime = new ElapsedTime();
                while (runtime.milliseconds() < 500) { if (!opMode.opModeIsActive()) return; }
                auxMotors.setPower(SHOOTER, .5);
                opMode.telemetry.addData("Status", "Restarted motor..."); opMode.telemetry.update();
                loadedSecond = true;
            }
        }
        auxMotors.setPower(SHOOTER, 0);
        auxMotors.setTargetPosition(SHOOTER, 0, 0);
        auxMotors.setMode(SHOOTER, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        auxMotors.setMode(SHOOTER, DcMotor.RunMode.RUN_TO_POSITION);
        servos.setPosition(LOADER, LOAD);
    }

    private static boolean compareTarget ( int[] current, int[] target ) {
        if (current.length != target.length) return false;
        boolean Return = true;
        for ( int i = 0; i < current.length; i++ ) {
            if (current[i] > target[i]+10 || current[i] < target[i]-10) {
                Return = false; break;
            }
        }
        return Return;
    }

}
