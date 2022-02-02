package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;

@Autonomous
public class ColorTest extends LinearOpMode {

    String telemData;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FreightSensor sensor = new FreightSensor(this);
        sensor.init(hardwareMap);

        waitForStart();

//        if (isStopRequested()) return;
        while(opModeIsActive()) {
            if (sensor.hasFreight()) {
                telemData = "True";
            } else {
                telemData = "False";
            }

            telemetry.addData("Detect Freight", telemData);
            telemetry.update();
        }
    }
}
