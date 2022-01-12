package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.HashMap;
import java.util.Map;

public class Drivetrain extends Mechanism {
    private static final double     COUNTS_PER_MOTOR_REV    = 537.7;
//    public FtcDashboard dash;
//    public TelemetryPacket packet;

    /**
     * Drivetrain gear ratio (< 1.0 if geared up).
     */
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;

    /**
     * Diameter of wheel in inches.
     */
    private static final double     WHEEL_DIAMETER_INCHES   = 1.8898 * 2;

    /**
     * Calculated ticks per inch.
     */
    private static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public double varPower = 0;
    public double varCorr = 0;

    //TODO make private again
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public BNO055IMU imu;
    public PIDController pidDrive;
    public PIDController pidRotate;
    public PIDController pidStrafe;

    double  globalAngle;
    Orientation lastAngles = new Orientation();

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.dcMotor.get("frontLeft");
        frontRight = hwMap.dcMotor.get("frontRight");
        backLeft = hwMap.dcMotor.get("backLeft");
        backRight = hwMap.dcMotor.get("backRight");

        //Set motor direction (AndyMark configuration)
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor brake behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidRotate = new PIDController(0.005, 0.1, 0);
        pidDrive = new PIDController(0.05,0,0);

        // Set all motors to zero power
        setPower(0.0);

        // Initialize IMU with parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private void setPower(double power) {
        setPower(power, power, power, power);
    }

    public void setPower(double FL, double FR, double BL, double BR) {
        frontLeft.setPower(FL);
        frontRight.setPower(FR);
        backLeft.setPower(BL);
        backRight.setPower(BR);
    }

    public void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        backRight.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
    }


    public void teleDrive(double r, double robotAngle, double rightX) {
        double v1 = -r * Math.sin(robotAngle) + rightX;
        double v2 = -r * Math.cos(robotAngle) - rightX;
        double v3 = -r * Math.cos(robotAngle) + rightX;
        double v4 = -r * Math.sin(robotAngle) - rightX;
        setPower(v1,v2,v3,v4);
    }

    public void strafeLeft() {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPower(-0.5, 0.5, 0.5, -0.5);
    }

    public void strafeRight() {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPower(0.5, -0.5, -0.5, 0.5);
    }

//    public void strafePID(double power, double duration) {
//        ElapsedTime time = new ElapsedTime();
//        time.reset();
//        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        pidStrafe.reset();
//        pidStrafe.setSetpoint(0);
//        pidStrafe.setOutputRange(0, power);
//        pidStrafe.setInputRange(-90, 90);
//        pidStrafe.enable();
//
//        double targetPower = 0.0;
//        int i = 0;
//        while(opMode.opModeIsActive() && (time.seconds() <= duration)) {
//            double corrections = pidStrafe.performPID(getAngle());
//
//            opMode.telemetry.addData("corrections",varCorr);
//            opMode.telemetry.addData("getAngle", getAngle());
//
//            targetPower = power;
//            frontLeft.setPower(targetPower - corrections);
//            backLeft.setPower(-targetPower - corrections);
//            backRight.setPower(targetPower + corrections);
//            frontRight.setPower(-targetPower + corrections);
//
//            opMode.telemetry.addData("Power: ", targetPower);
//            opMode.telemetry.addData("Angle: ", getAngle());
//            opMode.telemetry.addData("Correction:", corrections);
//
//
//            opMode.telemetry.update();
//        }
//
//        frontLeft.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//        frontRight.setPower(0);
//    }
//
//    public void driveToPos(double inches, double power) {
//        ElapsedTime time = new ElapsedTime();
//        time.reset();
//        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        int tickCount = (int) (inches * COUNTS_PER_INCH);
//        double set_power = power * inches/Math.abs(inches);
//
//        frontLeft.setTargetPosition(tickCount);
//        frontRight.setTargetPosition(tickCount);
//        backLeft.setTargetPosition(tickCount);
//        backRight.setTargetPosition(tickCount);
//
//        setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while(opMode.opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
//            driveStraightPID(set_power, inches);
//
//            if (time.seconds() > 3) {
//                setPower(0.0);
//                return;
//            }
//        }
//
//        setPower(0.0);
//        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void driveStraightPID(double power, double inches) {
//        double leftSpeed = -power, rightSpeed = -power;
//        pidDrive.setPID(.07, 0, 0);
//        // Set up parameters for driving in a straight line.
//
//        pidDrive.setSetpoint(0);
//        pidDrive.setOutputRange(0, power);
//        pidDrive.setInputRange(-90, 90);
//        pidRotate.enable();
//        double corrections = pidDrive.performPID(getAngle());
//        varCorr = corrections;
//
//
//        if (Math.signum(inches) >= 0) {
//            setPower(leftSpeed + corrections,  rightSpeed - corrections, leftSpeed + corrections, rightSpeed - corrections);
//        } else if (Math.signum(inches) < 0){
//            setPower(leftSpeed - corrections,  rightSpeed + corrections, leftSpeed - corrections, rightSpeed + corrections);
//        }
//    }
//
//    /**
//     * Resets the cumulative angle tracking to zero.
//     */
//    public void resetAngle() {
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        globalAngle = 0;
//    }
//
//    public float getHeading() {
//        return lastAngles.firstAngle;
//    }
//
//    public double getGlobal() {
//        return globalAngle;
//    }
//
//    /**
//     * Get current cumulative angle rotation from last reset.
//     * @return Angle in degrees. + = left, - = right from zero point.
//     */
//    public double getAngle()
//    {
//
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }
//
//    /**
//     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
//     * @param degrees Degrees to turn, + is left - is right
//     */
//    public void turn(int degrees, double power) {
//       turnPID (degrees,power, 0.02, 0.001, 0);
//    }
//
//    public void turnPID(int degrees, double power, double p, double i, double d) {
//        // restart imu angle tracking.
//        resetAngle();
//        pidRotate.setPID(p, i, d);
//        pidRotate.reset();
//        pidRotate.setSetpoint(degrees);
//        pidRotate.setInputRange(0, degrees);
//        pidRotate.setOutputRange(0, power);
//        pidRotate.setTolerance(1);
//        pidRotate.enable();
//        if (degrees < 0) {
////             Get it stuck off 0 degrees
//            while (getAngle() == 0) {
//                setPower(power, -power, power, -power);
////                 Make sure it moves a little bit
//                opMode.sleep(100);
//            }
//            do {
//                power = pidRotate.performPID(getAngle()); // power will be negative on right turn.
//                setPower(-power, power, -power, power);
//            }
//            while (!pidRotate.onTarget());
//        }
//        else    // left turn.
//            do {
//                power = pidRotate.performPID(getAngle()); // power will be positive on left turn.
//                setPower(-power, power, -power, power);
//            }
//            while (!pidRotate.onTarget());
//
//        setPower(0);
//        opMode.sleep(200);
////        ready for the next turn
//        resetAngle();
//    }
}
