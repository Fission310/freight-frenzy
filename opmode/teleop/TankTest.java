package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TankTest", group="Test")

public class TankTest extends LinearOpMode {
    // Set variables for the four motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double backLeftPower = 0;
    private double backRightPower = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        // Set the variables to their names in the configuration
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();


        while (opModeIsActive()) {
            // run until the end of the match (driver presses STOP)
            telemetry.addData("Status", "Running");
            telemetry.update();

            backLeftPower = -this.gamepad1.left_stick_y;
            backRightPower = this.gamepad1.right_stick_y;

            //Instructions for the Back Left Motor
            backLeft.setPower(backLeftPower);
            frontLeft.setPower(backLeftPower);
            telemetry.addData("Left Target Power", backLeftPower);
            telemetry.addData("Left Motor Power", backLeft.getPower());
            telemetry.update();

            //Instructions for the Back Right Motor
            frontRight.setPower(backRightPower);
            backRight.setPower(backRightPower);
            telemetry.addData("Right Target Power", backRightPower);
            telemetry.addData("Right Motor Power", backRight.getPower());
            telemetry.update();
        }
    }
}
