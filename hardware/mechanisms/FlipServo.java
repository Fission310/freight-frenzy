package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.util.Mechanism;

@Config
public class FlipServo extends Mechanism {
    Servo servo;

    DcMotorEx intake;

    public static double OPEN_POS = 1;
    public static double CLOSE_POS = 0;

    public FlipServo(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "grant");
        intake = hwMap.get(DcMotorEx.class, "intake");
    }

    public void open() {
        servo.setPosition(OPEN_POS);
    }

    public void close() {
        servo.setPosition(CLOSE_POS);
    }

    public void intake() {
        intake.setPower(1);
    }
    public void outtake() {
        intake.setPower(-1);
    }
    public void stop() {
        intake.setPower(0);
    }

    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            open();
        } else if (gamepad.dpad_down) {
            close();
        }

        if (gamepad.right_trigger > 0) {
            intake();
        } else if (gamepad.left_trigger > 0) {
            outtake();
        } else {
            stop();
        }
    }

}
