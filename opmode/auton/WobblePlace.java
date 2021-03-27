package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Wobble;

@Autonomous(name="WobblePlace", group="Test")
public class WobblePlace extends LinearOpMode{


    private Drivetrain drive = new Drivetrain(this);
    private Wobble wobble = new Wobble(this);
    private Camera camera = new Camera(this);
    private Camera.Ring rings;

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        wobble.init(hardwareMap);
        camera.init(hardwareMap);

        waitForStart();

        drive.driveToPos(-12, 0.4);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        while(time.seconds() < 4){
            rings = camera.listDetections();

            telemetry.addData("rings", rings);
            telemetry.update();
        }

        if(rings == Camera.Ring.ONE){
            drive.turn(90, 0.8);
        }
        else if(rings == Camera.Ring.FOUR){
            drive.turn(-90, 0.8);
        }
        telemetry.addData("rings", rings);
        telemetry.update();

        camera.shutdown();


    }
}
