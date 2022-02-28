package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Acquirer extends Mechanism {

    private DcMotor intakeRight;
    private DcMotor intakeLeft;

    public static double POWER = 0.85;

    ElapsedTime outtakeDelay = new ElapsedTime();
    ElapsedTime outtakeDuration = new ElapsedTime();

    public static double OUTTAKE_DELAY_TIME = 0.2;
    public static double OUTTAKE_DURATION_TIME = 0.5;

    public enum AcquirerState {
        ACQUIRER_START,
        ACQUIRER_DELAY,
        ACQUIRER_PREVENT
    }

    AcquirerState acquirerState = AcquirerState.ACQUIRER_START;

    public Acquirer(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        intakeRight = hwMap.dcMotor.get("intakeRight");
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeft = hwMap.dcMotor.get("intakeLeft");
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeDelay.reset();
        outtakeDuration.reset();
    }

    public void intake() {
        intakeRight.setPower(-POWER);
        intakeLeft.setPower(POWER);
    }
    public void outtake() {
        intakeRight.setPower(POWER);
        intakeLeft.setPower(-POWER);
    }
    public void stop() {
        intakeRight.setPower(0);
        intakeLeft.setPower(0);
    }

//    public boolean sensorStatus() { return freightSensor.hasFreight(); }

    public void loop(Gamepad gamepad1) {
        switch (acquirerState) {
            case ACQUIRER_START:
                if (gamepad1.right_trigger > 0) {
                    intake();
//                    if (freightSensor.hasFreight()) {
//                        acquirerState = AcquirerState.ACQUIRER_DELAY;
//                        outtakeDelay.reset();
//                    }
                } else if (gamepad1.left_trigger > 0) {
                    outtake();
                } else {
                    stop();
                }
                break;
            case ACQUIRER_DELAY:
                if (outtakeDelay.seconds() >= OUTTAKE_DELAY_TIME) {
                    acquirerState = AcquirerState.ACQUIRER_PREVENT;
                    outtakeDuration.reset();
                } else {
                    intake();
                }
                break;
            case ACQUIRER_PREVENT:
                if (outtakeDuration.seconds() >= OUTTAKE_DURATION_TIME) {
                    acquirerState = AcquirerState.ACQUIRER_START;
                } else {
                    outtake();
                }
        }
    }

}
