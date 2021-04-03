package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Flicker;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;

@Autonomous(name="RingShoot", group = "Test")
public class RingShoot extends LinearOpMode {

    private Drivetrain drive = new Drivetrain(this);
    private Flywheel flywheel = new Flywheel(this);
    private Acquirer acquirer = new Acquirer(this);
    private Flicker flicker = new Flicker(this);

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        flywheel.init(hardwareMap);
        acquirer.init(hardwareMap);
        flicker.init(hardwareMap);

        waitForStart();

        drive.driveToPos(-40, 0.8);
        drive.turn(10,0.8);

        flywheel.forward();
        sleep(1000);
        for(int i = 0; i < 3; i++){
            flicker.swing();
            sleep(500);
            flicker.reset();
            sleep(500);
        }

    }
}
