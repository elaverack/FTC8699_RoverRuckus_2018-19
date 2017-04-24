package org.firstinspires.ftc.teamcode.archive.season2016_2017.all_opmodes_120916;

import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.MotorHandler;
import org.firstinspires.ftc.teamcode.archive.season2016_2017.robotCoreFunctions.ServoHandler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Rama on 2/19/2016.
 */

public class Competition_Autonomous extends LinearOpMode {
    float speedLeft;
    float speedRight;
    float armLeft;
    float armRight;
    //ears starting position
    final double Leftarm_start = 0;
    final double Rightarm_start = 0;
    //to check tha the servo on horns is down
    boolean winchLock = false ;

    boolean dPadUp;
    boolean dPadDown;
    boolean aPressed;
    boolean bPressed;
    boolean xPressed;
    boolean yPressed;
    final String[] armNames = {"hookArm", "winchArm"};


    /*
    DcMotor motorRight1;
    DcMotor motorRight2;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    Servo LeftArm;
    Servo RightArm;
    */

    ServoHandler Arms;
    MotorHandler DriveTrain;
    MotorHandler arm;
    Servo winch;
    Servo bumper;
    int[] encoderPositions;

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();

        DriveTrain = new MotorHandler(hardwareMap, 4.5875f);
        Arms = new ServoHandler(hardwareMap);

        arm = new MotorHandler(hardwareMap, armNames, 1);
        winch = hardwareMap.servo.get("winchServo");

        bumper = hardwareMap.servo.get("bumper");

        DriveTrain.AddMotors(new String[]{"rf", "rb", "lf", "lb"});
        Arms.AddServo(new String[]{"leftArm", "rightArm"});

        DriveTrain.SetDirection(new String[]{"lf", "lb"}, DcMotor.Direction.REVERSE);
        DriveTrain.SetDirection(new String[]{"rf", "rb"}, DcMotor.Direction.FORWARD);

        Arms.SetServo(new String[]{"leftArm", "rightArm"}, new double[]{Leftarm_start, Rightarm_start});
        winch.setPosition(0);
        bumper.setPosition(0.6);

        DriveTrain.setAllMotorsPower(-1);
        sleep(6000);
        DriveTrain.setAllMotorsPower(0);
        sleep(10);

    }
}
