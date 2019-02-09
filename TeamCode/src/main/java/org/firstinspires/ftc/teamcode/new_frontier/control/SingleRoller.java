package org.firstinspires.ftc.teamcode.new_frontier.control;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums;

public class SingleRoller {
    
    private CRServo r;
    private ModernRoboticsI2cColorSensor c;
    private boolean
            //pulling = false,
            //pushing = false,
            //done = false,
            doout = false,
            //donepass = false,
            doin = false,
            dodep = false;
    private static final double INTAKE = .5, OUTTAKE = -.5, SILVER_TIME = .1;
    private Enums.Mineral grabbed = Enums.Mineral.ERROR;
    private ElapsedTime time;
    
    public SingleRoller (CRServo roller, ModernRoboticsI2cColorSensor color) {
        r = roller;
        r.setPower(0);
        c = color;
        c.enableLed(true);
        time = new ElapsedTime();
    }
    
    public void start () {
        r.setPower(INTAKE);
    }
    
    public void drive (boolean out, boolean in) {
        byte read = c.read8(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX);
        
        if (out && !doout && !(in || doin)) {
            r.setPower(OUTTAKE);
            doout = true;
        } else if (doout && !out) {
            r.setPower(INTAKE);
            grabbed = Enums.Mineral.ERROR;
//            pulling = false;
//            pushing = false;
//            done = false;
//            donepass = false;
            doout = false;
        }
        
        if (in && !doin && !(out || doout)) {
            r.setPower(INTAKE);
            doin = true;
        } else if (doin && !in) {
            grabbed = Enums.Mineral.ERROR;
//            pulling = false;
//            pushing = false;
//            done = false;
//            donepass = false;
            doin = false;
        }
        
        if (grabbed == Enums.Mineral.ERROR && read != 0 && !(doout || doin || dodep)) {
            if ((grabbed = Enums.Mineral.identify(read)) == Enums.Mineral.SILVER) {
                time.reset();
            } else r.setPower(0);
        }
        if (grabbed == Enums.Mineral.SILVER && time.seconds() > SILVER_TIME && r.getPower() == INTAKE && !(doout || doin || dodep)) r.setPower(0);
        if (grabbed != Enums.Mineral.ERROR && read == 0 && !(doout || doin || dodep)) {
            r.setPower(OUTTAKE / 1.5);
            grabbed = Enums.Mineral.ERROR;
        }
        
//        if (!done && !(doout || doin || in || out)) {
//            if (read != 0 && grabbed == Enums.Mineral.ERROR) { // if we are seeing something and don't already have something
//                pulling = true;
//                grabbed = Enums.Mineral.identify(read);
//            } else if (pulling && read == 0) { // if we are pulling one through and don't see it anymore
//                pulling = false;
//            } else if (!pulling && !pushing && read != 0) { // if we aren't pulling or pushing and we are seeing something
//                if (Enums.Mineral.identify(read) != grabbed) {
//                    r.setPower(OUTTAKE);
//                    pushing = true;
//                } else {
//                    r.setPower(INTAKE / 2);
//                    done = true;
//                }
//            } else if (pushing && Enums.Mineral.identify(read) == grabbed) { // if we are pushing one away and we are seeing the one we were holding
//                pulling = true;
//                pushing = false;
//                r.setPower(INTAKE);
//            }
//        } else if (done && !(doout || doin || in || out)) {
//            if (read == 0 && !donepass) {
//                r.setPower(OUTTAKE / 2);
//                donepass = true;
//            } else if (donepass) {
//                r.setPower(0);
//            }
//        }
    }
    
    public void drive (boolean deposit, int arm_pos, boolean out, boolean in, Arm arm) {
        byte read = c.read8(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX);
        
        if (deposit && !dodep && !(out || doout || in || doin)) {
            if (arm_pos == 4) r.setPower(OUTTAKE);
            else if (arm_pos == 3) r.setPower(INTAKE);
            dodep = true;
        } else if (dodep && !deposit) {
            r.setPower(INTAKE);
            grabbed = Enums.Mineral.ERROR;
            dodep = false;
        }
    
        if (out && !doout && !(in || doin || deposit || dodep)) {
            r.setPower(OUTTAKE);
            doout = true;
        } else if (doout && !out) {
            r.setPower(INTAKE);
            grabbed = Enums.Mineral.ERROR;
            doout = false;
        }
    
        if (in && !doin && !(out || doout || deposit || dodep)) {
            r.setPower(INTAKE);
            doin = true;
        } else if (doin && !in) {
            grabbed = Enums.Mineral.ERROR;
            doin = false;
        }
    
        if (grabbed == Enums.Mineral.ERROR && read != 0 && !(doout || doin || dodep)) {
            if ((grabbed = Enums.Mineral.identify(read)) == Enums.Mineral.SILVER) {
                time.reset();
            } else {
                r.setPower(0);
                arm.up_quick();
            }
        }
        if (grabbed == Enums.Mineral.SILVER && time.seconds() > SILVER_TIME && r.getPower() == INTAKE && !(doout || doin || dodep)) {
            r.setPower(0);
            arm.up_quick();
        }
        if (grabbed != Enums.Mineral.ERROR && read == 0 && !(doout || doin || dodep)) {
            if (grabbed == Enums.Mineral.GOLD) r.setPower(OUTTAKE / 1.5);
            else if (grabbed == Enums.Mineral.SILVER) r.setPower(INTAKE / 1.5);
            grabbed = Enums.Mineral.ERROR;
        }
    }
    
    public void autodrive () {
        byte read = c.read8(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX);
    
        if (grabbed == Enums.Mineral.ERROR && read != 0) {
            if ((grabbed = Enums.Mineral.identify(read)) == Enums.Mineral.SILVER) {
                time.reset();
            } else {
                r.setPower(0);
                //arm.up_quick();
            }
        }
        if (grabbed == Enums.Mineral.SILVER && time.seconds() > SILVER_TIME && r.getPower() == INTAKE) {
            r.setPower(0);
            //arm.up_quick();
        }
        if (grabbed != Enums.Mineral.ERROR && read == 0) {
            if (grabbed == Enums.Mineral.GOLD) r.setPower(OUTTAKE / 1.5);
            else if (grabbed == Enums.Mineral.SILVER) r.setPower(INTAKE / 1.5);
            grabbed = Enums.Mineral.ERROR;
        }
    }
    
    public Enums.Mineral getGrabbed () { return grabbed; }
    
    public void stop () {
        r.setPower(0);
        c.enableLed(false);
    }
    
}
