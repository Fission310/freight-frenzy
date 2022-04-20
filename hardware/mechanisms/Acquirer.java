package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.stuyfission.fissionlib.util.Mechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Acquirer extends Mechanism {

    private DcMotor intakeRight;
    private DcMotor intakeLeft;

    private FreightSensor sensor;

    public static double LEFT_POWER = 1;
    public static double RIGHT_POWER = 1;

    ElapsedTime outtakeDelay = new ElapsedTime();
    ElapsedTime outtakeDuration = new ElapsedTime();

    public static double OUTTAKE_DELAY_TIME = 0.12;
    public static double OUTTAKE_DURATION_TIME = 0.8;

    public enum AcquirerState {
        ACQUIRER_START,
        ACQUIRER_DELAY_LEFT,
        ACQUIRER_DELAY_RIGHT,
        ACQUIRER_PREVENT_LEFT,
        ACQUIRER_PREVENT_RIGHT
    }
    AcquirerState acquirerState;

    private boolean leftOuttake = false;

    public Acquirer(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private int i = 0;

    @Override
    public void init(HardwareMap hwMap) {
        intakeRight = hwMap.dcMotor.get("intakeRight");
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeft = hwMap.dcMotor.get("intakeLeft");
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        sensor = new FreightSensor(opMode);
        sensor.init(hwMap);

        acquirerState = AcquirerState.ACQUIRER_START;

        outtakeDelay.reset();
        outtakeDuration.reset();
    }

    public void intake() {
        intakeRight.setPower(RIGHT_POWER);
        intakeLeft.setPower(LEFT_POWER);

//        leftOuttake = false;
    }
    public void intakeLeft() {
        intakeLeft.setPower(LEFT_POWER);
    }
    public void intakeRight() {
        intakeRight.setPower(RIGHT_POWER);
    }

    public void outtakeLeft() {
        intakeLeft.setPower(-LEFT_POWER);
    }

    public void outtakeRight() {
        intakeRight.setPower(-RIGHT_POWER);
    }

    public void outtake(){
        intakeRight.setPower(-RIGHT_POWER);
        intakeLeft.setPower(-LEFT_POWER);
    }


    public void stop() {
        intakeRight.setPower(0);
        intakeLeft.setPower(0);
    }

//    public boolean sensorStatus() { return freightSensor.hasFreight(); }

    @Override
    public void telemetry(Telemetry telemetry){

//        if(sensor.hasFreight())i++;

        telemetry.addData("Left Sensor", sensor.hasFreightLeft());
        telemetry.addData("Right Sensor", sensor.hasFreightRight());

        telemetry.addData("leftOuttake", leftOuttake);
//        telemetry.addData("Delay", outtakeDelay.seconds());
//        telemetry.addData("Times", i);



        sensor.telemetry(telemetry);

    }

    public void autonLoop(){
        switch (acquirerState){
            case ACQUIRER_START:
//                leftOuttake = false;
                intake();
                if (sensor.hasFreightLeft()) {
//                    leftOuttake = true;


                    acquirerState = AcquirerState.ACQUIRER_DELAY_LEFT;
                    outtakeDelay.reset();
                }
                if  (sensor.hasFreightRight()) {
//                    leftOuttake = false;


                    acquirerState = AcquirerState.ACQUIRER_DELAY_RIGHT;
                    outtakeDelay.reset();
                }
                break;

            case ACQUIRER_DELAY_LEFT:
                if (outtakeDelay.seconds() >= OUTTAKE_DELAY_TIME) {
                    acquirerState = AcquirerState.ACQUIRER_PREVENT_LEFT;
                    outtakeDuration.reset();
                }
                else {
                    intake();
                }
                break;
            case ACQUIRER_PREVENT_LEFT:
                if (outtakeDuration.seconds() >= OUTTAKE_DURATION_TIME) {
                    acquirerState = AcquirerState.ACQUIRER_START;
                } else {
                    outtakeLeft();
                    intakeRight();
                }
                break;
            case ACQUIRER_PREVENT_RIGHT:
                if (outtakeDuration.seconds() >= OUTTAKE_DURATION_TIME) {
                    acquirerState = AcquirerState.ACQUIRER_START;
                } else {
                    outtakeRight();
                    intakeLeft();
                }
                break;
        }
    }

    @Override
    public void loop(Gamepad gamepad) {

        switch (acquirerState) {
            case ACQUIRER_START:
                if (gamepad.right_trigger > 0) {
                    intake();

                    if (sensor.hasFreightLeft()) {
//                        leftOuttake = true;
                        gamepad.rumble(200);

                        acquirerState = AcquirerState.ACQUIRER_DELAY_LEFT;
                        outtakeDelay.reset();
                    }
                    else if  (sensor.hasFreightRight()) {
//                        leftOuttake = false;

                        gamepad.rumble(200);

                        acquirerState = AcquirerState.ACQUIRER_DELAY_RIGHT;
                        outtakeDelay.reset();
                    }

                } else if (gamepad.left_trigger > 0) {
                    outtake();
                } else {
                    stop();
                }
                break;
            case ACQUIRER_DELAY_LEFT:
                if (outtakeDelay.seconds() >= OUTTAKE_DELAY_TIME) {
                    acquirerState = AcquirerState.ACQUIRER_PREVENT_LEFT;
                    outtakeDuration.reset();
                }
                else {
                    intake();
                }
                break;
            case ACQUIRER_DELAY_RIGHT:
                if (outtakeDelay.seconds() >= OUTTAKE_DELAY_TIME) {
                    acquirerState = AcquirerState.ACQUIRER_PREVENT_RIGHT;
                    outtakeDuration.reset();
                }
                else {
                    intake();
                }
                break;
            case ACQUIRER_PREVENT_LEFT:
                if (outtakeDuration.seconds() >= OUTTAKE_DURATION_TIME) {
                    acquirerState = AcquirerState.ACQUIRER_START;
                } else {
                    outtakeLeft();
                    intakeRight();
                }
                break;
            case ACQUIRER_PREVENT_RIGHT:
                if (outtakeDuration.seconds() >= OUTTAKE_DURATION_TIME) {
                    acquirerState = AcquirerState.ACQUIRER_START;
                } else {
                    outtakeRight();
                    intakeLeft();
                }
                break;
        }
    }

}
