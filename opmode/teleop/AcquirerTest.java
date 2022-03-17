package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;

@TeleOp (name = "AcquirerTest", group = "Test")
public class AcquirerTest extends LinearOpMode {
    private Acquirer acquirer = new Acquirer(this);

    @Override
    public void runOpMode() throws InterruptedException {
        acquirer.init(hardwareMap);

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            acquirer.loop(gamepad1);
//            acquirer.telemetry(telemetry);
//            telemetry.update();
        }
    }
}
