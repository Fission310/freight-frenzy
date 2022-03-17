package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slides extends Mechanism {


    // ======= SERVOS ======= //
    private Servo leftSlide;
    private Servo rightSlide;
    private Servo freightServo;

    // ====================== //

    // ======= SPOOL ======= //
    public DcMotorEx spool;

    private double targetPosition;

    // PID constants //
    public static double EXTEND_POS = 11;
    public static double MAX_VEL = 10;
    public static double MAX_ACCEL = 10;
    private static double WHEEL_RADIUS = 1.37795;
    private static double TICKS_PER_REV = 537.6;
    private static double GEAR_RATIO = 1.0;

    public static double kF = 0;
    public static PIDCoefficients coeffs = new PIDCoefficients(0.2, 0, 0);

    PIDFController controller = new PIDFController(coeffs, 0, 0, 0, (position, velocity) -> kF);
    MotionProfile profile;

    public static ElapsedTime profileTimer = new ElapsedTime();
    // ============ //

    public enum SlidesState {
        SLIDES_START,
    }
    SlidesState slidesState;

    public Slides(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(Servo.class, "leftSlide");
        rightSlide = hwMap.get(Servo.class, "rightSlide");
        freightServo = hwMap.get(Servo.class, "freightServo");
        leftSlide.setDirection(Servo.Direction.FORWARD);
        rightSlide.setDirection(Servo.Direction.REVERSE);
        freightServo.setDirection(Servo.Direction.REVERSE);

        spool = hwMap.get(DcMotorEx.class, "slidesMotor");
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spool.setDirection(DcMotorSimple.Direction.REVERSE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setTargetPosition(0);

        slidesState = SlidesState.SLIDES_START;

    }

    // PID methods //
    public double getPosition() {
        return encoderTicksToInches(spool.getCurrentPosition());
    }
    public double getVelocity() {
        return encoderTicksToInches(spool.getVelocity());
    }

    public MotionProfile generateProfile(double target){
        MotionState start = new MotionState(getPosition(), getVelocity(), 0, 0);
        MotionState goal = new MotionState(targetPosition, 0, 0, 0);

        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }

    public void update() {
        MotionState state = profile.get(profileTimer.seconds());

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        double power = controller.update(getPosition(), getVelocity());

        spool.setPower(power);
    }

    public void setTargetPosition(double target){
        targetPosition = target;
        profile = generateProfile(targetPosition);
        profileTimer.reset();
    }
    // ========= //



    public void loop(Gamepad gamepad){
        if(gamepad.b){
            setTargetPosition(EXTEND_POS);
        }
        else if(gamepad.a){
            setTargetPosition(0);
        }
        update();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Velocity", getVelocity());
        telemetry.addData("Position", getPosition());
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