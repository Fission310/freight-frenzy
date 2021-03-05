package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Flicker;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;


@TeleOp(name="PairedMain", group="Main")
public class PairedMain extends LinearOpMode {

    private double leftInput1, rightInput1, slideInput1, leftInput2, rightInput2, slideInput2;
    private Drivetrain drive = new Drivetrain(this);
    private Acquirer acquirer = new Acquirer(this);
    private Flicker flicker = new Flicker(this);
    private Flywheel flywheel = new Flywheel(this);

    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing
        acquirer.init(hardwareMap);
        drive.init(hardwareMap);
        flicker.init(hardwareMap);

        boolean rotate = true;

        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }
        waitForStart();

        while(opModeIsActive()) {
            //Inputs for the stick and triggers
            // Sticks are [0,1], triggers are [-1,1] as a sum
            leftInput1 = gamepad1.left_stick_y;
            rightInput1 = gamepad1.right_stick_y;
            slideInput1 = -gamepad1.left_trigger + gamepad1.right_trigger;

            //Second controller
            leftInput2 = -gamepad2.left_stick_y;
            rightInput2 = -gamepad2.right_stick_y;
            slideInput2 = -gamepad2.left_trigger + gamepad2.right_trigger;

            //How far the stick goes
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

            //\scuffed way to smooth inputs by cubing
            r = Math.pow(r,3);
            leftInput2 = Math.pow(leftInput2, 3);

            //Angle of the stick
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

            //How far the right stick goes from side to side (turning)
            double rightX1 = gamepad1.right_stick_x;

            // Controller 1: Driver
            if (gamepad1.dpad_left) {
                drive.strafeLeft();
            }
            else if (gamepad1.dpad_right) {
                drive.strafeRight();
            }
            else if (slideInput1 < -0.3) {
                drive.teleDrive(r/3, robotAngle, rightX1/3);
            }
            else {
                drive.teleDrive(r, robotAngle, rightX1);
            }

            if(gamepad2.x){
                flicker.swing();
                sleep(500);
                flicker.reset();
            }

            if(gamepad2.y){
                rotate = !rotate;
            }
            if(rotate){
                flywheel.forward();

            }
            else{
                flywheel.reverse();
            }

            if(gamepad2.right_bumper){
                acquirer.forward();
            }
            else if(gamepad2.left_bumper){
                acquirer.reverse();
            }
            else{
                acquirer.stop();
            }





            telemetry.addData("angle", drive.getAngle());
//            telemetry.addData("slide", slideInput1);
//            telemetry.addData("righttrigger", gamepad1.right_trigger);
//            telemetry.addData("rightx1", rightX1);
//            telemetry.addData("leftx1", gamepad1.left_stick_x);
//            telemetry.addData("lefty", leftInput2);
            telemetry.update();

        }
    }
}