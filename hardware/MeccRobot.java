package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Carousel;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.OdoLift;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Slides;

public class MeccRobot extends Mechanism {

    // init all mechanisms
    private Drivetrain dt = new Drivetrain(opMode);
    private Acquirer acquirer = new Acquirer(opMode);
    private Carousel carousel = new Carousel(opMode);
    private OdoLift odoLift = new OdoLift(opMode);
    private Slides slides = new Slides(opMode);

    public MeccRobot(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        dt.init(hwMap);
        acquirer.init(hwMap);
        carousel.init(hwMap);
        odoLift.init(hwMap);
        slides.init(hwMap);

        odoLift.lift();
    }


    public void loop(Gamepad gamepad1) {
        dt.loop(gamepad1);
        acquirer.loop(gamepad1);
        carousel.loop(gamepad1);
        odoLift.loop(gamepad1);
        slides.loop(gamepad1);
    }

    public void telemetry(Telemetry telemetry){
        slides.telemetry(telemetry);
    }

}
