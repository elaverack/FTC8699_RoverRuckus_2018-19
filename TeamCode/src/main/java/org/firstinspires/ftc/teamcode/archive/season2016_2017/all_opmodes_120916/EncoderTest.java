package org.firstinspires.ftc.teamcode.archive.season2016_2017.all_opmodes_120916;

import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.DirectionHandler;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.MotorHandler;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.ServoHandler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftccommon.DbgLog;

import java.text.MessageFormat;

/**
 * Created by Chandler on 1/17/2016.
 */
public class EncoderTest extends LinearOpMode {

    MotorHandler driveTrain;
    ServoHandler arms;
    DirectionHandler dH;
    final double leftarm_start = 0;
    final double rightarm_start = 1;
    int[] encoderPositions = new int[4];
    private final String[] allMotors = new String[]{"rf", "rb", "lf", "lb"};
    private final String[] leftMotors = new String[]{"lf", "lb"};
    private final String[] rightMotors = new String[]{"rf", "rb"};
    private final String[] armNames = new String[]{"leftArm", "rightArm"};


    public void runOpMode() throws InterruptedException {

        waitOneFullHardwareCycle();

        driveTrain = new MotorHandler(hardwareMap, allMotors, 4);
        arms = new ServoHandler(hardwareMap, armNames);
        dH = new DirectionHandler(driveTrain, allMotors);

        driveTrain.SetDirection(leftMotors, DcMotor.Direction.FORWARD);
        driveTrain.SetDirection(rightMotors, DcMotor.Direction.REVERSE);

        arms.SetServo(armNames, new double[]{leftarm_start, rightarm_start});

        waitForStart();

        doTest();
        //testDistance();
        //testTurning();
    }

    public String getMotorPositionsString() {
        String s = "RF: " + driveTrain.getMotor("rf").getCurrentPosition();
        s += ", RB: " + driveTrain.getMotor("rb").getCurrentPosition();
        s += ", LF: " + driveTrain.getMotor("lf").getCurrentPosition();
        s += ", LB: " + driveTrain.getMotor("lb").getCurrentPosition();
        return s;
    }

    public void testDistance() throws InterruptedException {
        driveTrain.moveDistanceEncoder(12f, allMotors);
        driveTrain.setMotorsPower(allMotors, 1d);

        waitOneFullHardwareCycle();
        sleep(500);
        DbgLog.msg(getMotorPositionsString());

        DbgLog.msg("resetting encoders");
        driveTrain.setAllRunModes(DcMotor.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        sleep(500);
        DbgLog.msg(getMotorPositionsString());
    }

    public void testTurning() throws InterruptedException {
        DbgLog.msg("turning once"); // just in case
        turn();
        sleep(2000);

        DbgLog.msg("resetting encoders"); // just in case
        driveTrain.setAllRunModes(DcMotor.RunMode.RESET_ENCODERS);
        DbgLog.msg(getMotorPositionsString());

        DbgLog.msg("turning twice");
        turn();
    }

    protected void turn() throws InterruptedException {
        dH.turnByAngle(90, true);
        driveTrain.setMotorsPower(allMotors, 1d);
        waitOneFullHardwareCycle();
        sleep(500);
        DbgLog.msg(getMotorPositionsString());
        driveTrain.setMotorsPower(allMotors, 0d);
    }

    public void doTest() throws InterruptedException {
        DbgLog.msg(MessageFormat.format("doTest 1: Moving {0} via encoder for {1} inches...", allMotors, 12f));
        int goal = driveTrain.moveDistanceEncoder(12f, allMotors);

        driveGuaranteed(allMotors, goal, 10);

        sleep(2000);

        DbgLog.msg(MessageFormat.format("doTest 2: Turning {0} via encoder for {1} degrees...", allMotors, 90));
        dH.turnByAngle(90, true);

        turnGuaranteed(rightMotors, driveTrain.rightGoal, leftMotors, driveTrain.leftGoal, 10);
    }

    private void driveGuaranteed(String[] motorNames, int goal, int range) throws InterruptedException {
        DbgLog.msg("goal = " + goal);
        while (!driveTrain.checkAvg(motorNames, goal, range)) {
            driveTrain.setAllMotorsPower(1);
            DbgLog.msg(MessageFormat.format("waitForMotors: Not at position, Goal is {0}", driveTrain.checkAvg(motorNames, goal, range)));
            telemetry.addData("Motor Set: ", driveTrain.checkAvg(motorNames, goal, range));
            sleep(20);
        }
        DbgLog.msg(MessageFormat.format("waitForMotors: AT POSITION, Goal is {0}", driveTrain.checkAvg(motorNames, goal, range)));
        DbgLog.msg(MessageFormat.format("waitForMotors: Giving test; {0} is at {1}", motorNames[0], driveTrain.readEncoder(motorNames[0])));
        driveTrain.setAllMotorsPower(0);
    }

    private void turnGuaranteed(String[] motorNames1, int goal1, String[] motorNames2, int goal2, int range) throws InterruptedException {
        while (!(driveTrain.checkAvg(motorNames1, goal1, range) && driveTrain.checkAvg(motorNames2, goal2, range))) {
            driveTrain.setAllMotorsPower(1);
            DbgLog.msg(MessageFormat.format("waitForMotors: Not at position, Goal1 is {0}, Goal2 is {1}", driveTrain.checkAvg(motorNames1, goal1, range), driveTrain.checkAvg(motorNames2, goal2, range)));
            telemetry.addData("Motor Set 1: ", driveTrain.checkAvg(motorNames1, goal1, range));
            telemetry.addData("Motor Set 2: ", driveTrain.checkAvg(motorNames2, goal2, range));
            sleep(20);
        }
        DbgLog.msg(MessageFormat.format("waitForMotors: AT POSITION; Goal1 is {0}, Goal2 is {1}", driveTrain.checkAvg(motorNames1, goal1, range), driveTrain.checkAvg(motorNames2, goal2, range)));
        DbgLog.msg(MessageFormat.format("waitForMotors: Giving tests; {0} is at {1} and {2} is at {3}", motorNames1[0], driveTrain.readEncoder(motorNames1[0]), motorNames2[0], driveTrain.readEncoder(motorNames2[0])));
        driveTrain.setAllMotorsPower(0);
    }
}
