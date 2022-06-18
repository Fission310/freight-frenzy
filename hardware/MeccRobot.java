package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Carousel;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.Carriage;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.SlideMechanism;

public class MeccRobot extends Mechanism {

    public MeccRobot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private Drivetrain dt = new Drivetrain(opMode);
    private Acquirer acquirer = new Acquirer(opMode);
    private Slides slides = new Slides(opMode);
    private Carousel carousel = new Carousel(opMode);

    @Override
    public void init(HardwareMap hwMap) {
        dt.init(hwMap);
        acquirer.init(hwMap);
        slides.init(hwMap);
        carousel.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        dt.loop(gamepad);
        acquirer.loop(gamepad);
        slides.loop(gamepad);
        carousel.loop(gamepad);
    }

    public void telemetry(Telemetry telemetry){
//        acquirer.telemetry(telemetry);
//        slides.telemetry(telemetry);
//        slideMech.telemetry(telemetry);
    }
}
