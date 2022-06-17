package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.MeccRobot;

@TeleOp (name = "SingleMain", group = "drive")
public class SingleMain extends LinearOpMode {

    private MeccRobot robot = new MeccRobot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.loop(gamepad1);
            robot.telemetry(telemetry);

            telemetry.update();
        }
    }

}
