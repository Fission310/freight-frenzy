package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Config
public class SlidesTest extends Mechanism {

    public DcMotorEx testMotor;

    public static int SLIDE_EXTEND_POS = 700;
    public static int SLIDE_RETRACT_POS = 0;

    public enum MotorState {
        MOTOR_EXTENDED,
        MOTOR_RETRACTED
    }

    MotorState motorState = MotorState.MOTOR_RETRACTED;

    public SlidesTest(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        testMotor = hwMap.get(DcMotorEx.class, "testMotor");
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testMotor.setTargetPositionTolerance(10);
    }

    // 537 ticks per rev
    public void extend() {
        testMotor.setPower(0.5);
        testMotor.setTargetPosition(SLIDE_EXTEND_POS ); // 1 revolution
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void retract() {
        testMotor.setPower(0.5);
        testMotor.setTargetPosition(SLIDE_RETRACT_POS);
        testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getPos() {
        return testMotor.getCurrentPosition(); // in ticks
    }

    public int getTarget() {
        return testMotor.getTargetPosition(); // in ticks
    }



    public void loop(Gamepad gamepad1) {
//        if(!testMotor.isBusy()) testMotor.setPower(0);

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
    }
}
