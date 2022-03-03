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

    public static double PERP_LIFT_HEIGHT = 0.75;
    public static double PERP_LOWER_HEIGHT = 0;

    public static double HUB_LIFT_HEIGHT = 0.65;
    public static double HUB_LOWER_HEIGHT = 0;

    public static double WALL_LIFT_HEIGHT = 0.8;
    public static double WALL_LOWER_HEIGHT = 0;

    public OdoLift (LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        wallOdoServo = hwMap.get(Servo.class, "wallOdoServo");
        hubOdoServo = hwMap.get(Servo.class, "hubOdoServo");
        perpOdoServo = hwMap.get(Servo.class, "perpOdoServo");

        hubOdoServo.setDirection(Servo.Direction.FORWARD);
        hubOdoServo.setDirection(Servo.Direction.REVERSE);
        perpOdoServo.setDirection(Servo.Direction.REVERSE);
    }

    public void lift() {
        wallOdoServo.setPosition(WALL_LIFT_HEIGHT);
        hubOdoServo.setPosition(HUB_LIFT_HEIGHT);
        perpOdoServo.setPosition(PERP_LIFT_HEIGHT);
    }
    public void lower() {
        wallOdoServo.setPosition(WALL_LOWER_HEIGHT);
        hubOdoServo.setPosition(HUB_LOWER_HEIGHT);
        perpOdoServo.setPosition(PERP_LOWER_HEIGHT);
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
