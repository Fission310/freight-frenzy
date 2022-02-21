package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.CarouselTest;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.CarriageSensor;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidesTest;

public class MeccRobot extends Mechanism {

    // init all mechanisms
    private Drivetrain dt = new Drivetrain(opMode);
    private Acquirer acquirer = new Acquirer(opMode);
    private CarouselTest supaspeed = new CarouselTest(opMode);
    private CarriageSensor carriageSensor = new CarriageSensor(opMode);
    private Slides slides = new Slides(opMode);
    private SlidesTest slidesTest = new SlidesTest(opMode);

//    private MultipleTelemetry telemetry;

    public MeccRobot(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        dt.init(hwMap);
        acquirer.init(hwMap);
        supaspeed.init(hwMap);
        carriageSensor.init(hwMap);
        slides.init(hwMap);
        slidesTest.init(hwMap);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Distance MM", carriageSensor.distanceMM());
        telemetry.addData("Distance CM", carriageSensor.distanceCM());
    }

    public void loop(Gamepad gamepad1) {
        dt.loop(gamepad1);
        acquirer.loop(gamepad1);
        supaspeed.loop(gamepad1);
        slides.loop(gamepad1);
        slidesTest.loop(gamepad1);
    }

}
