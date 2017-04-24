package org.firstinspires.ftc.teamcode.archive.season2016_2017.all_opmodes_120916;


import android.content.res.Configuration;

import com.qualcomm.ftccommon.DbgLog;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.DirectionHandler;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.MotorHandler;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.MountainDetector;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.ServoHandler;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.TiltDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.text.MessageFormat;

public abstract class AbstractAutonomousOpMode extends LinearOpMode {

    MotorHandler driveTrain;
    ServoHandler arms;
    DirectionHandler dH;
    final double leftarm_start = 0;
    final double rightarm_start = 1;
    int[] encoderPositions = new int[4];
    private String[] allMotors = new String[]{"rf", "rb", "lf", "lb"};
    private final String[] leftMotors = new String[]{"lf", "lb"};
    private final String[] rightMotors = new String[]{"rf", "rb"};
    private final String[] armNames = new String[]{"leftArm", "rightArm"};
    MountainDetector md;
    TiltDetector td;

    public abstract float getFirstForwardAmount();

    public abstract int getFirstTurnAngle();

    // return either R or B
    public abstract char getTeamColor();

    public boolean useTilt = false;

    @Override
    public void runOpMode() throws InterruptedException {
        waitOneFullHardwareCycle();
        DbgLog.msg("starting AutoBaseOpMode");
        // DbgLog.msg("hardwareMap.appContext: "+hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE));

        initElectronics();

        waitForStart();
        DbgLog.msg("about to drive forward");
        try {
            drive(getFirstForwardAmount()); // implemented in subclasses
        } catch (Exception e) {
            DbgLog.error("problem driving: " + e.getMessage());
            DbgLog.logStacktrace(e);
            driveTrain.setAllMotorsPower(0);
        }

        try {
            turn(getFirstTurnAngle()); // implemented in subclasses
        } catch (Exception e) {
            DbgLog.error("problem turning: " + e.getMessage());
            DbgLog.logStacktrace(e);
            driveTrain.setAllMotorsPower(0);
        }

        DbgLog.msg("about to start looking for mountain");
        while (opModeIsActive()) {
            // if tilting, fix that first
            float roll = td.getRoll();
            DbgLog.msg("tilt = " + roll);
            // turn right or left if tilting
            if (useTilt && Math.abs(roll) > 15) {
                if (roll < -15) { // tilting left!
                    DbgLog.msg("turning right due to left tilt");
                    turn(30); //turn 30 degrees right
                } else if (roll > 15) {
                    DbgLog.msg("turning left due to right tilt");
                    turn(-30); //turn 30 degrees left
                }
            } else {
                // if not tilting, find and climb mountain
                int loc = md.getLocation(getTeamColor());
                DbgLog.msg("location is " + loc);
                // turn right or left based on loc
                if (loc == Integer.MAX_VALUE) {
                    // no red/blue found -- hunt for mtn
                    turn(-20); // arbitrary number - maybe should be different for each side of field
                }
                if (Math.abs(loc) > 15) { // an arbitrary threshold of 15 - anything below that can be considered center
                    telemetry.addData("turning ", loc);
                    turn(loc);
                }
            }
            telemetry.addData("driving fwd ","4\"");
            DbgLog.msg("driving forward 4\"");
            drive(4f); // arbitrary - 4" at a time
        }
    }

    protected void drive(float distance) throws InterruptedException {
        int goal = driveTrain.moveDistanceEncoder(distance, allMotors);
        driveGuaranteed(allMotors, goal, 10);
    }

    protected void turn(int angle) throws InterruptedException {
        dH.turnByAngle(angle, true);
        turnGuaranteed(rightMotors, driveTrain.rightGoal, leftMotors, driveTrain.leftGoal, 10);
    }

    protected void initElectronics() {

        driveTrain = new MotorHandler(hardwareMap, allMotors, 4.5875f);
        arms = new ServoHandler(hardwareMap, new String[]{"leftArm", "rightArm"});
        dH = new DirectionHandler(driveTrain, allMotors);

        driveTrain.SetDirection(new String[]{"lf", "lb"}, DcMotor.Direction.FORWARD);
        driveTrain.SetDirection(new String[]{"rf", "rb"}, DcMotor.Direction.REVERSE);

        arms.SetServo(armNames, new double[]{leftarm_start, rightarm_start});

        // init phone sensors
        FtcRobotControllerActivity a = (FtcRobotControllerActivity) hardwareMap.appContext;
        md = new MountainDetector(a, this, Configuration.ORIENTATION_LANDSCAPE);
        td = new TiltDetector(this.hardwareMap, Configuration.ORIENTATION_LANDSCAPE);
    }

    protected void driveGuaranteed(String[] motorNames, int goal, int range) throws InterruptedException {
        DbgLog.msg("goal = " + goal);
        while (!driveTrain.checkAvg(motorNames, goal, range)) {
            driveTrain.setAllMotorsPower(1);
            //DbgLog.msg(MessageFormat.format("waitForMotors: Not at position, Goal is {0}", driveTrain.checkAvg(motorNames, goal, range)));
            telemetry.addData("Motor Set: ", driveTrain.checkAvg(motorNames, goal, range));
            sleep(20);
        }
        DbgLog.msg(MessageFormat.format("AT POSITION, Goal is {0}", driveTrain.checkAvg(motorNames, goal, range)));
        DbgLog.msg(MessageFormat.format("Giving test; {0} is at {1}", motorNames[0], driveTrain.readEncoder(motorNames[0])));
        driveTrain.setAllMotorsPower(0);
    }

    protected void turnGuaranteed(String[] motorNames1, int goal1, String[] motorNames2, int goal2, int range) throws InterruptedException {
        while (!(driveTrain.checkAvg(motorNames1, goal1, range) && driveTrain.checkAvg(motorNames2, goal2, range))) {
            driveTrain.setAllMotorsPower(1);
            //DbgLog.msg(MessageFormat.format("waitForMotors: Not at position, Goal1 is {0}, Goal2 is {1}", driveTrain.checkAvg(motorNames1, goal1, range), driveTrain.checkAvg(motorNames2, goal2, range)));
            //telemetry.addData("Motor Set 1: ", driveTrain.checkAvg(motorNames1, goal1, range));
            //telemetry.addData("Motor Set 2: ", driveTrain.checkAvg(motorNames2, goal2, range));
            sleep(20);
        }
        DbgLog.msg(MessageFormat.format("waitForMotors: AT POSITION; Goal1 is {0}, Goal2 is {1}", driveTrain.checkAvg(motorNames1, goal1, range), driveTrain.checkAvg(motorNames2, goal2, range)));
        DbgLog.msg(MessageFormat.format("waitForMotors: Giving tests; {0} is at {1} and {2} is at {3}", motorNames1[0], driveTrain.readEncoder(motorNames1[0]), motorNames2[0], driveTrain.readEncoder(motorNames2[0])));
        driveTrain.setAllMotorsPower(0);
    }

}
