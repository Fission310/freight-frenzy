package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Acquirer extends Mechanism {
    private Servo leftHub;
    private Servo leftWall;
    private Servo rightHub;
    private Servo rightWall;

//    private DcMotorEx intakeLeft;
    private DcMotorEx intakeRight;

    private FreightSensor sensor;

    public static double LEFT_UP_POS = 0.25;
    public static double LEFT_UP_INACTIVE = 0.15;
    public static double LEFT_DOWN_POS = 0.03;

    public static double RIGHT_UP_POS = 0.25;
    public static double RIGHT_UP_INACTIVE = 0.15;
    public static double RIGHT_DOWN_POS = 0.03;

    public static double FLIP_DELAY = 0;
    public static double OUTTAKE_DELAY = 0.25;
    public static double RESET_DELAY = 1.25;

    public static double SPEED_OUTTAKE = 0.7;

    private ElapsedTime timer = new ElapsedTime();

    public Acquirer(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public enum AcquirerState {
        ACQUIRER_START_RIGHT,
        ACQUIRER_START_LEFT,
        ACQUIRER_FLIPPING_LEFT,
        ACQUIRER_FLIPPING_RIGHT,
        ACQUIRER_OUTTAKING_LEFT,
        ACQUIRER_OUTTAKING_RIGHT

    }
    AcquirerState acquirerState;

    @Override
    public void init(HardwareMap hwMap) {
        leftWall = hwMap.get(Servo.class, "leftWall");
        leftHub = hwMap.get(Servo.class, "leftHub");
        rightWall = hwMap.get(Servo.class, "rightWall");
        rightHub = hwMap.get(Servo.class, "rightHub");

        leftWall.setDirection(Servo.Direction.REVERSE);
        leftHub.setDirection(Servo.Direction.FORWARD);
        rightWall.setDirection(Servo.Direction.FORWARD);
        rightHub.setDirection(Servo.Direction.REVERSE);


//        intakeLeft = hwMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hwMap.get(DcMotorEx.class, "intakeRight");

        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        sensor = new FreightSensor(opMode);
        sensor.init(hwMap);

        acquirerState = AcquirerState.ACQUIRER_START_RIGHT;
        timer.reset();

        raiseLeft();
        raiseRight();

    }

    public void raiseLeft() {
        leftHub.setPosition(LEFT_UP_POS);
        leftWall.setPosition(LEFT_UP_POS);
    }
    public void raiseLeftInactive() {
        leftHub.setPosition(LEFT_UP_INACTIVE);
        leftWall.setPosition(LEFT_UP_INACTIVE);
    }

    public void raiseRight() {
        rightHub.setPosition(RIGHT_UP_POS);
        rightWall.setPosition(RIGHT_UP_POS);
    }
    public void raiseRightInactive() {
        rightHub.setPosition(RIGHT_UP_INACTIVE);
        rightWall.setPosition(RIGHT_UP_INACTIVE);
    }

    public void lowerLeft() {
        leftHub.setPosition(LEFT_DOWN_POS);
        leftWall.setPosition(LEFT_DOWN_POS);
    }

    public void lowerRight() {
        rightHub.setPosition(RIGHT_DOWN_POS);
        rightWall.setPosition(RIGHT_DOWN_POS);
    }

//    public void intakeLeft() {
//        intakeLeft.setPower(1);
//    }

    public void intakeRight() {
        intakeRight.setPower(1);
    }

    public void intake(){
//        intakeLeft.setPower(1);
        intakeRight.setPower(1);
    }

//    public void outtakeLeft() {
//    intakeLeft.setPower(SPEED_OUTTAKE);
//    }

    public void outtakeRight() {
        intakeRight.setPower(SPEED_OUTTAKE);
    }

//    public void stopLeft() {
//        intakeLeft.setPower(0);
//    }

    public void stopRight() {
        intakeRight.setPower(0);
    }

    public void stop() {
        intakeRight.setPower(0);
//        intakeLeft.setPower(0);
    }

    public void autonLoop(){

        return;
    }

    public void loop(Gamepad gamepad) {
        switch (acquirerState) {
            case ACQUIRER_START_RIGHT:
                if (gamepad.right_trigger > 0) {
                    raiseLeftInactive();
                    lowerRight();
                    intakeRight();

                } else if(gamepad.left_trigger > 0){
                    acquirerState = AcquirerState.ACQUIRER_START_LEFT;
                    break;
                }
                else {
                    stop();
                }

                if (sensor.hasFreightRight()) {
                    gamepad.rumble(200);

                    raiseRight();
                    stop();

                    acquirerState = AcquirerState.ACQUIRER_FLIPPING_RIGHT;
                    timer.reset();
                }



                break;
            case ACQUIRER_START_LEFT:
                if (gamepad.right_trigger > 0) {

                    acquirerState = AcquirerState.ACQUIRER_START_RIGHT;
                    break;

                } else if(gamepad.left_trigger > 0){
                    raiseRightInactive();
                    lowerLeft();
//                    intakeLeft();
                }
                else {
                    stop();
                }

                if (sensor.hasFreightLeft()) {
                    gamepad.rumble(200);

                    raiseLeft();
                    stop();

                    acquirerState = AcquirerState.ACQUIRER_FLIPPING_LEFT;
                    timer.reset();
                }
                break;

            case ACQUIRER_FLIPPING_RIGHT:
                if(timer.seconds() >= OUTTAKE_DELAY){
                    outtakeRight();

                    acquirerState = AcquirerState.ACQUIRER_OUTTAKING_RIGHT;
                    timer.reset();
                }
                break;
            case ACQUIRER_FLIPPING_LEFT:
                if(timer.seconds() >= OUTTAKE_DELAY){
//                    outtakeLeft();

                    acquirerState = AcquirerState.ACQUIRER_OUTTAKING_LEFT;
                    timer.reset();
                }
                break;
            case ACQUIRER_OUTTAKING_RIGHT:
                if(timer.seconds() >= RESET_DELAY){
                    lowerRight();
                    stop();

                    acquirerState = AcquirerState.ACQUIRER_START_RIGHT;
                    timer.reset();
                }
                break;
            case ACQUIRER_OUTTAKING_LEFT:
                if(timer.seconds() >= RESET_DELAY){
                    lowerLeft();
                    stop();

                    acquirerState = AcquirerState.ACQUIRER_START_LEFT;
                    timer.reset();
                }
                break;


        }


    }

    public void telemetry(Telemetry telemetry){

        telemetry.addData("Left Sensor", sensor.hasFreightLeft());
        telemetry.addData("Right Sensor", sensor.hasFreightRight());

//        telemetry.addData("Delay", timer.seconds());

//        telemetry.addData("state", acquirerState);



        sensor.telemetry(telemetry);

    }

}
