package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.CarouselTest;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.hardware.SlidesTest;

@Config
@TeleOp (name = "SingleMain", group = "Test")
public class SingleMain extends LinearOpMode {

    private SampleMecanumDrive rrDrive;

    // ===== TESTING ===== //
    private Slides slides = new Slides(this);
    private Acquirer acquirer = new Acquirer(this);
    private SlidesTest slidesTest = new SlidesTest(this);
    private CarouselTest supaspeed = new CarouselTest(this);
    // =================== //

    @Override
    public void runOpMode() throws InterruptedException {

        rrDrive = new SampleMecanumDrive(hardwareMap);

        slides.init(hardwareMap);
        acquirer.init(hardwareMap);
        slidesTest.init(hardwareMap);
        supaspeed.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            rrDrive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            slides.loop(gamepad1);
            acquirer.loop(gamepad1);
            slidesTest.loop(gamepad1);

            if (gamepad1.right_bumper) {
                supaspeed.rotate();
            } else if (gamepad1.left_bumper) {
                supaspeed.reverse();
            } else {
                supaspeed.stop();
            }

            telemetry.addData("Cup pos", slides.getCupPos());
            telemetry.addData("Cup timer", slides.getCupTimer());
//            telemetry.addData("Motor position", slidesTest.getPos());
//            telemetry.addData("Target position", slidesTest.getTarget());
<<<<<<< Updated upstream
            telemetry.addData("Busy?", slidesTest.testMotor.isBusy());
=======
            telemetry.addData("Has Freight?", acquirer.sensorStatus());
>>>>>>> Stashed changes
            telemetry.update();

        }
    }

}
