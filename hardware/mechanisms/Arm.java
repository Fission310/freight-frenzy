package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Arm extends Mechanism {
    private Servo arm;

    public static double TIP = 0.5;
    public static double RESET = 1;

    ElapsedTime armDelay = new ElapsedTime();

    public static double ARM_DELAY_TIME = 1;

    public enum ArmState {
        ARM_TIP,
        ARM_RESET
    }

    ArmState armState = ArmState.ARM_RESET;

    public Arm(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        arm = hwMap.get(Servo.class, "arm");

        armDelay.reset();
    }

    public void tip() {
        arm.setPosition(TIP);
    }
    public void reset() {
        arm.setPosition(RESET);
    }

    public void loop(Gamepad gamepad1) {
        switch (armState) {
            case ARM_RESET:
                reset();
                if (gamepad1.y) {
                    armState = ArmState.ARM_TIP;
                    armDelay.reset();
                    tip();
                }
                break;
            case ARM_TIP:
                if (armDelay.seconds() >= ARM_DELAY_TIME) {
                    armState = ArmState.ARM_RESET;
                    armDelay.reset();
                    reset();
                }
        }
    }
}
