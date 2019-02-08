package org.firstinspires.ftc.teamcode.new_frontier.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.frontier.control.Latcher;
import org.firstinspires.ftc.teamcode.new_frontier.control.ToggleServo;

// Created on 1/19/2019 at 4:50 PM by Chandler, originally part of ftc_app under org.firstinspires.ftc.teamcode.frontier.teleop.test

@TeleOp(name = "Frontier2", group = "test")
//@Disabled
public class Frontier2 extends OpMode {
    
    DcMotor rf, rb, lf, lb, lift;
    ToggleServo lock;
    Latcher latcher;
//    LayoutInterfacer layout;
//    int joyS_ID = 314159, joyT = 3141592;
//    double leftx = 0, lefty = 0, rightx = 0, righty = 0;
    double maxPower = .6;
    
    @Override
    public void init() {
    
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
    
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
    
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
        //lift = hardwareMap.dcMotor.get("l");
        //lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        latcher = new Latcher(hardwareMap.dcMotor.get("l"));
    
        lock = new ToggleServo(hardwareMap.servo.get("lock"), .85, .95);
        
        // FRONT IMU IS GOOD
        
        
        
        //ImageView iv = new ImageView(om.hardwareMap.appContext);
        //int       ID = 314159;
        //iv.setId(ID);
    
//        layout = new LayoutInterfacer(this, (LinearLayout)((Activity)hardwareMap.appContext).findViewById(R.id.monitorContainer));
//
//        layout.run(new Runnable() {
//            @Override
//            public void run () {
//                JoystickView
//                        j1 = new JoystickView(hardwareMap.appContext),
//                        j2 = new JoystickView(hardwareMap.appContext);
//                j1.setOnMoveListener(new JoystickView.OnMoveListener() {
//                    @Override
//                    public void onMove (int angle, int strength) {
//                        leftx = -Math.cos(Math.toRadians(angle))*(((double)strength)/100);
//                        lefty = Math.sin(Math.toRadians(angle))*(((double)strength)/100);
//                    }
//                });
//                j1.setBackgroundColor(Color.GRAY);
//                j1.setButtonSizeRatio(.5f);
//                j1.setButtonSizeRatio(.1f);
//                j2.setOnMoveListener(new JoystickView.OnMoveListener() {
//                    @Override
//                    public void onMove (int angle, int strength) {
//                        rightx = -Math.cos(Math.toRadians(angle))*(((double)strength)/100);
//                        righty = Math.sin(Math.toRadians(angle))*(((double)strength)/100);
//                    }
//                });
//                j2.setBackgroundColor(Color.GRAY);
//                layout.createViews(j1, j2);
//            }
//        });
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() { lock.open(); }

    @Override
    public void loop() {
        
        if (gamepad1.right_bumper) maxPower = 1;
        else maxPower = .6;
    
        double powerRF, powerRB, powerLF, powerLB;
        double
                straight    = -gamepad1.right_stick_y,
                strafe      = gamepad1.right_stick_x,
                rotate      = gamepad1.left_stick_x;
        powerRF = straight;
        powerRB = straight;
        powerLF = straight;
        powerLB = straight;
        powerRF -= strafe;
        powerRB += strafe;
        powerLF += strafe;
        powerLB -= strafe;
        powerRF -= rotate;
        powerRB -= rotate;
        powerLF += rotate;
        powerLB += rotate;
        if (powerRF > 1) {powerRF = 1f;} else if (powerRF < -1) {powerRF = -1f;}
        if (powerRB > 1) {powerRB = 1f;} else if (powerRB < -1) {powerRB = -1f;}
        if (powerLF > 1) {powerLF = 1f;} else if (powerLF < -1) {powerLF = -1f;}
        if (powerLB > 1) {powerLB = 1f;} else if (powerLB < -1) {powerLB = -1f;}
    
        if (gamepad1.right_trigger > 0.5) { powerLB /= 4f; powerLF /= 4f; powerRB /= 4f; powerRF /= 4f; }
    
        lf.setPower(powerLF*maxPower);
        lb.setPower(powerLB*maxPower);
        rf.setPower(powerRF*maxPower);
        rb.setPower(powerRB*maxPower);
    
//        if (gamepad1.right_bumper && !gamepad1.left_bumper) lift.setPower(.5);
//        else if (gamepad1.left_bumper && !gamepad1.right_bumper) lift.setPower(-.5);
//        else lift.setPower(0);
        latcher.run(
                gamepad1.dpad_up,
                gamepad1.dpad_down,
                gamepad1.left_bumper,
                gamepad1.left_trigger > .5,
                gamepad1.right_stick_button && gamepad1.left_stick_button);
        
        lock.tob(gamepad1.x);
        
        telemetry.addData("lb", lf.getCurrentPosition());
        telemetry.addData("lf", lb.getCurrentPosition());
        telemetry.addData("rf", rf.getCurrentPosition());
        telemetry.addData("rb", rb.getCurrentPosition());
        //telemetry.addData("lift", lift.getCurrentPosition());
        
    }

    @Override
    public void stop() {
    
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
        //lift.setPower(0);
        latcher.stop();
        
    }

}