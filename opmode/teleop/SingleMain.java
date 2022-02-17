package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.Lift;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="SingleMain", group="Test")
public class SingleMain extends LinearOpMode{

    public static int reverseDelay = 200;
    public static int reverseDuration = 1000;

    private Drivetrain drive = new Drivetrain(this);
    private Acquirer acquirer = new Acquirer(this);
    private Carousel carousel = new Carousel(this);
    private Lift lift = new Lift(this);
    private FreightSensor sensor = new FreightSensor(this);

    private SampleMecanumDrive rrDrive;

    private boolean intaking;

    @Override
    public void runOpMode() throws InterruptedException{
        ScheduledExecutorService service = Executors.newSingleThreadScheduledExecutor();

        Runnable lower = new Runnable() {
            @Override
            public void run() {
                lift.toggleSlide();
            }
        };

        Runnable resetArm = new Runnable() {
            @Override
            public void run() {
                if(lift.closed) lift.toggleRoof();

                if(lift.tipped) lift.toggleCup();
                else if(lift.low) lift.toggleShared();

                service.schedule(lower, 1000, TimeUnit.MILLISECONDS);
            }
        };


        Runnable resetIntake = new Runnable() {
            @Override
            public void run() {
                intaking = true;
            }
        };

        Runnable tempReverse = new Runnable() {
            @Override
            public void run() {
                intaking = false;

                service.schedule(resetIntake, reverseDuration, TimeUnit.MILLISECONDS);
            }


        };

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.init(hardwareMap);
        acquirer.init(hardwareMap);
        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        sensor.init(hardwareMap);

        rrDrive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime slideWait = new ElapsedTime();
        ElapsedTime cupWait = new ElapsedTime();
        ElapsedTime roofWait = new ElapsedTime();

        waitForStart();

        intaking = true;

        slideWait.reset();
        cupWait.reset();
        roofWait.reset();


        while(opModeIsActive() && !isStopRequested()){

            Pose2d poseEstimate = rrDrive.getPoseEstimate();

//            //Inputs for the stick and triggers
//            // Sticks are [0,1], triggers are [-1,1] as a sum
//            double slideInput = -gamepad1.left_trigger + gamepad1.right_trigger;
//
//            //How far the stick goes
//            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//
//            //Scuffed way to smooth inputs by cubing
//            r = Math.pow(r, 3);
//
//            //Angle of the stick
//            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//
//            //How far the right stick goes from side to side (turning)
//            double rightX = gamepad1.right_stick_x;
//
//            // Strafing
//            if (gamepad1.dpad_left)  drive.strafeLeft();
//            else if (gamepad1.dpad_right)  drive.strafeRight();
//
//            // Slow and regular driving
//            else if (gamepad1.right_trigger > 0.3) drive.teleDrive(r / 3, robotAngle, rightX / 3);
//            else drive.teleDrive(r , robotAngle, rightX );


            rrDrive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );



            //Acquirer
            if(gamepad1.right_trigger > 0) {
                if (intaking) acquirer.acquire();
                else acquirer.reverse();
            }
            else if(gamepad1.left_trigger > 0) {
                acquirer.reverse();
            }
            else acquirer.stop();

            //Carousel
            if(gamepad1.a) carousel.spin();
            else if(gamepad1.b) carousel.reverse();
            else carousel.stop();

            //Lift
            if(gamepad1.y && slideWait.seconds() > 0.5 && !(sensor.hasFreight() && lift.raised))  {

                lift.toggleSlide();
                slideWait.reset();
            }
            if(gamepad1.x && cupWait.seconds() > 0.5) {
                lift.toggleCup();
                cupWait.reset();

                if(lift.tipped)service.schedule(resetArm, 1000, TimeUnit.MILLISECONDS);

            }

            if(gamepad1.right_bumper && cupWait.seconds() > 0.5){
                lift.toggleShared();
                cupWait.reset();

                

                if(lift.low)service.schedule(resetArm, 1000, TimeUnit.MILLISECONDS);
            }

            //Roof
            if(gamepad1.dpad_up && roofWait.seconds() > 0.5){
                lift.toggleRoof();
                roofWait.reset();
            }






            if (sensor.hasFreight()) {

                service.schedule(tempReverse, reverseDelay, TimeUnit.MILLISECONDS);
//                intaking = false;

                if(!lift.raised){
                    gamepad1.rumble(250);
                    lift.toggleSlide();

                    if(!lift.closed) lift.toggleRoof();
                }


            } else {


//                intaking = true;
            }


            telemetry.addData("Detect Freight", sensor.hasFreight());
            telemetry.addData("Raised?", lift.raised);
            telemetry.addData("Tipped?", lift.tipped);
            telemetry.addData("Low?", lift.low);
            telemetry.addData("Closed?", lift.closed);
            telemetry.update();




        }

    }
}
