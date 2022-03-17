package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.TestDistSensor;

@TeleOp (name = "DistanceTest", group = "Test")
public class DistanceTest extends LinearOpMode {

    private TestDistSensor distSensor = new TestDistSensor(this);

    @Override
    public void runOpMode() throws InterruptedException {
        distSensor.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            distSensor.telemetry(telemetry);
            telemetry.update();
        }
    }
}
