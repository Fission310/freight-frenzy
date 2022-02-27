package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.CarouselTest;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.OdoLift;

@Config
@TeleOp (name = "FFTest", group = "Test")
public class FFTest extends LinearOpMode {
    CarouselTest carousel = new CarouselTest(this);
    OdoLift odoLift = new OdoLift(this);
    @Override
    public void runOpMode() throws InterruptedException {
        carousel.init(hardwareMap);
        odoLift.init(hardwareMap);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            carousel.loop(gamepad1);
            odoLift.loop(gamepad1);
        }
    }
}
