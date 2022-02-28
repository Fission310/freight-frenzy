package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OdoLift extends Mechanism {
    private Servo wallOdoServo;
    private Servo hubOdoServo;
    private Servo perpOdoServo;

    public static double LIFT_HEIGHT = 1;
    public static double LOWER_HEIGHT = 0;

    public OdoLift (LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        wallOdoServo = hwMap.get(Servo.class, "wallOdoServo");
        hubOdoServo = hwMap.get(Servo.class, "hubOdoServo");
        perpOdoServo = hwMap.get(Servo.class, "perpOdoServo");
    }

    public void lift() {
        wallOdoServo.setPosition(LIFT_HEIGHT);
        hubOdoServo.setPosition(LIFT_HEIGHT);
        perpOdoServo.setPosition(LIFT_HEIGHT);
    }
    public void lower() {
        wallOdoServo.setPosition(LOWER_HEIGHT);
        hubOdoServo.setPosition(LOWER_HEIGHT);
        perpOdoServo.setPosition(LOWER_HEIGHT);
    }

    public void loop(Gamepad gamepad1) {
        if (gamepad1.dpad_up) {
            lift();
        }
        if (gamepad1.dpad_down) {
            lower();
        }
    }
}
