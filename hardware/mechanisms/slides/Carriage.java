package org.firstinspires.ftc.teamcode.hardware.mechanisms.slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Carriage extends Mechanism {
    Servo cup;
    Servo armLeft;
    Servo armRight;

    public static double CUP_LEVEL3 = 0.35;
    public static double CUP_LEVEL3_TIP = 0.7;
    public static double CUP_LEVEL2 = 0;
    public static double CUP_LEVEL2_TIP = 0.55;
    public static double CUP_LEVEL1 = 0.9;
    public static double CUP_LEVEL1_TIP = 1;
    public static double CUP_REST = 0.79;

    public static double ARM_LEVEL3 = 0.80;
    public static double ARM_LEVEL2 = 1;
    public static double ARM_LEVEL1 = 0;
    public static double ARM_REST = 0.23;

    public static double CUP_TEST = 0.77;
    public static double ARM_TEST = 0.21;

    public Carriage(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        cup = hwMap.get(Servo.class, "freightServo");
        cup.setDirection(Servo.Direction.REVERSE);

        armLeft = hwMap.get(Servo.class, "leftSlide");
        armLeft.setDirection(Servo.Direction.FORWARD);

        armRight = hwMap.get(Servo.class, "rightSlide");
        armRight.setDirection(Servo.Direction.REVERSE);
    }

    public void rest() {
        armLeft.setPosition(ARM_REST);
        armRight.setPosition(ARM_REST);
        cup.setPosition(CUP_REST);
    }

    public void level3Temp() {
        armLeft.setPosition(ARM_LEVEL3);
        armRight.setPosition(ARM_LEVEL3);
        cup.setPosition(CUP_LEVEL3);
    }
    public void level3Tip() {
        armLeft.setPosition(ARM_LEVEL3);
        armRight.setPosition(ARM_LEVEL3);
        cup.setPosition(CUP_LEVEL3_TIP);
    }
    public void level2Temp(){
        armLeft.setPosition(ARM_LEVEL2);
        armRight.setPosition(ARM_LEVEL2);
        cup.setPosition(CUP_LEVEL2);
    }
    public void level2Tip(){
        armLeft.setPosition(ARM_LEVEL2);
        armRight.setPosition(ARM_LEVEL2);
        cup.setPosition(CUP_LEVEL2_TIP);
    }

    public void level1Temp(){
        armLeft.setPosition(ARM_LEVEL1);
        armRight.setPosition(ARM_LEVEL1);
        cup.setPosition(CUP_LEVEL1);
    }
    public void level1Tip() {
        armLeft.setPosition(ARM_LEVEL1);
        armRight.setPosition(ARM_LEVEL1);
        cup.setPosition(CUP_LEVEL1_TIP);
    }

//    @Override
//    public void loop(Gamepad gamepad) {
//        if (gamepad.x) {
//            rest();
//        }
//        if (gamepad.dpad_up) {
//            lvl3();
//        }
//        if (gamepad.dpad_down) {
//            lvl1();
//        }
//    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("arm servo pos", armLeft.getPosition());
        telemetry.addData("cup servo pos", cup.getPosition());
    }
}