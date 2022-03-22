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

    public static double CUP_MAX = 0.7;
    public static double CUP_REST = 0.08;
    public static double CUP_MIN = 0;

    public static double ARM_MAX = 1;
    public static double ARM_REST = 0.21;
    public static double ARM_MIN = 0;

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

    public void lvl3() {
        armLeft.setPosition(ARM_MAX);
        armRight.setPosition(ARM_MAX);
        //TODO: cup.setPosition
    }

    public void lvl1() {
        armLeft.setPosition(ARM_MIN);
        armRight.setPosition(ARM_MIN);
        //TODO: cup.setPosition
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.x) {
            rest();
        }
        if (gamepad.dpad_up) {
            lvl3();
        }
        if (gamepad.dpad_down) {
            lvl1();
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("arm servo pos", armLeft.getPosition());
        telemetry.addData("cup servo pos", cup.getPosition());
    }
}
