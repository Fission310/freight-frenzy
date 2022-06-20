package org.firstinspires.ftc.teamcode.hardware.mechanisms.slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.FreightSensor;

@Config
public class Carriage extends Mechanism {
    Servo cup;
    Servo armLeft;
    Servo armRight;
    FreightSensor sensor;

    public static double CUP_CLOSED = 0.56;
    public static double CUP_OPEN = 0;

    public static double ARM_LEVEL3 = 0.8;
    public static double ARM_LEVEL2 = 0.92;
    public static double ARM_LEVEL1 = 1;

    public static double ARM_LOWER_CAP = 1;
    public static double ARM_MID_CAP = 0.75;
    public static double ARM_HIGHER_CAP = 0.6;

    public static double ARM_REST = 0;

    public static double CUP_TEST = 0;

    public Carriage(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        cup = hwMap.get(Servo.class, "clamp");
        cup.setDirection(Servo.Direction.REVERSE);

        armLeft = hwMap.get(Servo.class, "leftArm");
        armLeft.setDirection(Servo.Direction.FORWARD);

        armRight = hwMap.get(Servo.class, "rightArm");
        armRight.setDirection(Servo.Direction.REVERSE);

        sensor = new FreightSensor(opMode);
        sensor.init(hwMap);

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

    public void level2(){
        armLeft.setPosition(ARM_LEVEL2);
        armRight.setPosition(ARM_LEVEL2);
        cup.setPosition(CUP_CLOSED);
    }

    public void level1() {
        armLeft.setPosition(ARM_LEVEL1);
        armRight.setPosition(ARM_LEVEL1);
        cup.setPosition(CUP_CLOSED);
    }

    public boolean hasFreight() {
        return sensor.hasFreightClamp();
    }

    public void lowerCap() {
        armLeft.setPosition(ARM_LOWER_CAP);
        armRight.setPosition(ARM_LOWER_CAP);
        cup.setPosition(CUP_CLOSED);
    }
    public void midCap() {
        armLeft.setPosition(ARM_MID_CAP);
        armRight.setPosition(ARM_MID_CAP);
        cup.setPosition(CUP_CLOSED);
    }
    public void higherCap() {
        armLeft.setPosition(ARM_HIGHER_CAP);
        armRight.setPosition(ARM_HIGHER_CAP);
        cup.setPosition(CUP_CLOSED);
    }

    @Override
    public void loop(Gamepad gamepad2) {
        if (gamepad2.y) {
            higherCap();
        }
        if (gamepad2.b) {
            midCap();
        }
        if (gamepad2.a) {
            lowerCap();
        }
        if (gamepad2.x) {
            rest();
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("arm servo pos", armLeft.getPosition());
        telemetry.addData("cup servo pos", cup.getPosition());
    }
}
