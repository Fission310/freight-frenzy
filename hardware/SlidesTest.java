package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class SlidesTest extends Mechanism {

    public DcMotorEx testMotor;

    public static int SLIDE_EXTEND_POS = 700;
    public static int SLIDE_RETRACT_POS = 0;

    public static double WHEEL_RADIUS = 1.81102;
    public static double TICKS_PER_REV = 145.1;
    public static double GEAR_RATIO = 1.0;

    public static double targetPosition;

    public static PIDCoefficients coeffs = new PIDCoefficients(0.1, 0, 0);
    PIDFController controller;

    public enum MotorState {
        MOTOR_EXTENDED,
        MOTOR_RETRACTED
    }
    MotorState motorState;

    public SlidesTest(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        testMotor = hwMap.get(DcMotorEx.class, "testMotor");
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorState = MotorState.MOTOR_RETRACTED;
        targetPosition = SLIDE_RETRACT_POS;
        controller = new PIDFController(coeffs);
    }

    public void extend() {
        controller.setTargetPosition(SLIDE_EXTEND_POS);
    }
    public void retract() {
        controller.setTargetPosition(SLIDE_RETRACT_POS);
    }

    public double getPosition() {

        return encoderTicksToInches(testMotor.getCurrentPosition());
    }
    public double getVelocity() {

        return encoderTicksToInches(testMotor.getVelocity());
    }

    public void update(){
        double power = controller.update(getPosition(), getVelocity());
        testMotor.setPower(power);
    }



    public void loop(Gamepad gamepad1) {

        switch (motorState) {
            case MOTOR_RETRACTED:
                if (gamepad1.b && !testMotor.isBusy()) {
                    extend();
                    motorState = MotorState.MOTOR_EXTENDED;
                }
                break;
            case MOTOR_EXTENDED:
                if (gamepad1.b && !testMotor.isBusy()) {
                    retract();
                    motorState = MotorState.MOTOR_RETRACTED;
                }
                break;
            default:
                motorState = MotorState.MOTOR_RETRACTED;
        }

        update();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
