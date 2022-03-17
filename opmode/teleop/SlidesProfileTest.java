package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidesTest;

@TeleOp(name = "SlidesProfileTest", group = "Test")
public class SlidesProfileTest extends LinearOpMode {
    SlidesTest slidesTest = new SlidesTest(this);

    @Override
    public void runOpMode() throws InterruptedException {
        slidesTest.init(hardwareMap);

        waitForStart();
        
        while (opModeIsActive() && !isStopRequested()) {
            slidesTest.loop(gamepad1);
        }
    }
}
