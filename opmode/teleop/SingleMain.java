package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@TeleOp(name="SingleMain", group="Test")
public class SingleMain extends LinearOpMode{

    private Drivetrain drive = new Drivetrain(this);
    private Acquirer acquirer = new Acquirer(this);
    private Carousel carousel = new Carousel(this);
    private Lift lift = new Lift(this);

    @Override
    public void runOpMode() throws InterruptedException{

        drive.init(hardwareMap);
        acquirer.init(hardwareMap);
        carousel.init(hardwareMap);
        lift.init(hardwareMap);

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
            double rightX = gamepad1.right_stick_x;

            // Strafing
            if (gamepad1.dpad_left)  drive.strafeLeft();
            else if (gamepad1.dpad_right)  drive.strafeRight();

            // Slow and regular driving
            else if (gamepad1.right_trigger > 0.3) drive.teleDrive(r / 3, robotAngle, rightX / 3);
            else drive.teleDrive(r, robotAngle, rightX);


            //Carousel
            if(gamepad1.dpad_up) carousel.spin();
            else if(gamepad1.dpad_down) carousel.reverse();
            else carousel.stop();

            //Acquirer
            if(gamepad1.right_trigger > 0) acquirer.acquire();
            else if(gamepad1.left_trigger > 0) acquirer.reverse();
            else acquirer.stop();

            //Lift
            if(gamepad1.a) lift.reset();
            else if(gamepad1.x) lift.low();
            else if(gamepad1.y) lift.mid();
            else if(gamepad1.b) lift.high();

            if(gamepad1.right_bumper) lift.tip();
            else lift.down();




        }

    }
}
