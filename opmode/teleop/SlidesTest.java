package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Slides;

@TeleOp(name = "SlidesTest", group = "Test")
public class SlidesTest extends LinearOpMode {
    Slides slides = new Slides(this);

    @Override
    public void runOpMode() throws InterruptedException {
        slides.init(hardwareMap);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        
        while (opModeIsActive() && !isStopRequested()) {
            slides.loop(gamepad1);
            slides.telemetry(telemetry);
            telemetry.update();
        }
    }
}
