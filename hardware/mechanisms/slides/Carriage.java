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

    public static double CUP_CLOSED = 0.12;
    public static double CUP_OPEN = 0.7;

    public static double ARM_LEVEL3 = 0.85;
    public static double ARM_LEVEL2 = 0.95;
    public static double ARM_LEVEL1 = 1;
    public static double ARM_SHARED = 1;
    public static double ARM_REST = 0.006;

    public static double CUP_TEST = 0.77;
    public static double ARM_TEST = 0.21;

    public Carriage(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        cup = hwMap.get(Servo.class, "clamp");
        cup.setDirection(Servo.Direction.REVERSE);

        armLeft = hwMap.get(Servo.class, "leftArm");
        armLeft.setDirection(Servo.Direction.FORWARD);

        armRight = hwMap.get(Servo.class, "rightArm");
        armRight.setDirection(Servo.Direction.REVERSE);
    }

    public void rest() {
        armLeft.setPosition(ARM_REST);
        armRight.setPosition(ARM_REST);
        cup.setPosition(CUP_OPEN);
    }

    public void open(){
        cup.setPosition(CUP_OPEN);
    }

    public void close(){
        cup.setPosition(CUP_CLOSED);
    }

    public void level3() {
        armLeft.setPosition(ARM_LEVEL3);
        armRight.setPosition(ARM_LEVEL3);
        cup.setPosition(CUP_CLOSED);
    }

//    public void level3Close() {
//        armLeft.setPosition(ARM_LEVEL3_CLOSE);
//        armRight.setPosition(ARM_LEVEL3_CLOSE);
//        cup.setPosition(CUP_CLOSED);
//    }

    public void level2(){
        armLeft.setPosition(ARM_LEVEL2);
        armRight.setPosition(ARM_LEVEL2);
        cup.setPosition(CUP_CLOSED);
    }

    public void sharedTemp() {
        armLeft.setPosition(ARM_SHARED);
        armRight.setPosition(ARM_SHARED);
        cup.setPosition(CUP_CLOSED);
    }

//    public void sharedArmRestTemp() {
//        armLeft.setPosition(ARM_SHARED_TEMP);
//        armRight.setPosition(ARM_SHARED_TEMP);
//    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            armLeft.setPosition(ARM_REST);
            armRight.setPosition(ARM_REST);
        }
        if (gamepad.y) {
            armLeft.setPosition(ARM_LEVEL3);
            armRight.setPosition(ARM_LEVEL3);
        }
        if (gamepad.x) {
            armLeft.setPosition(ARM_LEVEL2);
            armRight.setPosition(ARM_LEVEL2);
        }
        if (gamepad.a) {
            armLeft.setPosition(ARM_LEVEL1);
            armRight.setPosition(ARM_LEVEL1);
        }
        if (gamepad.dpad_down) {
            cup.setPosition(CUP_TEST);
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("arm servo pos", armLeft.getPosition());
        telemetry.addData("cup servo pos", cup.getPosition());
    }
}
