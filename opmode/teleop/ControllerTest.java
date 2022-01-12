package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@Config
@TeleOp(name="ControllerTest", group="Test")
public class ControllerTest extends LinearOpMode{

    private Drivetrain drive = new Drivetrain(this);

    @Override
    public void runOpMode() throws InterruptedException{

        drive.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            //Inputs for the stick and triggers
            // Sticks are [0,1], triggers are [-1,1] as a sum
            double slideInput = -gamepad1.left_trigger + gamepad1.right_trigger;

            //How far the stick goes
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            //Scuffed way to smooth inputs by cubing
            r = Math.pow(r, 3);

            //Angle of the stick
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

            //How far the right stick goes from side to side (turning)
            double rightX1 = gamepad1.right_stick_x;

            // Controller 1: Driver
            if (gamepad1.dpad_left) {
                drive.strafeLeft();
            } else if (gamepad1.dpad_right) {
                drive.strafeRight();
            } else if (slideInput < -0.3) {
                drive.teleDrive(r / 3, robotAngle, rightX1 / 3);
            } else {
                drive.teleDrive(r, robotAngle, rightX1);
            }

        }

    }
}