package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Flicker;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Wobble;

@Autonomous(name="WobblePlace", group="Test")
public class WobblePlace extends LinearOpMode{


    private Drivetrain drive = new Drivetrain(this);
    private Wobble wobble = new Wobble(this);
    private Camera camera = new Camera(this);
    private Flywheel flywheel = new Flywheel(this);
    private Flicker flicker = new Flicker(this);
    private Camera.Ring rings;

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        wobble.init(hardwareMap);
        camera.init(hardwareMap);
        flywheel.init(hardwareMap);
        flicker.init(hardwareMap);

        waitForStart();

        drive.driveToPos(-10, 0.8);

        ElapsedTime time = new ElapsedTime();
        time.reset();
        while(time.seconds() < 4){
            rings = camera.listDetections();

            if(rings != Camera.Ring.NONE) break;
        }
        telemetry.addData("rings", rings);
        telemetry.update();

        drive.strafePID(-0.8, 0.72);
        drive.driveToPos(-22, 0.8);
        drive.turn(10,0.8);

        flywheel.forward();
        sleep(1000);
        for(int i = 0; i < 3; i++){
            flicker.swing();
            sleep(500);
            flicker.reset();
            sleep(500);
        }
        flywheel.stop();
        drive.turn(-10, 0.8);

        if(rings == Camera.Ring.NONE){
            wobble.rotateDown();
            sleep(500);
            wobble.open();

            drive.driveToPos(-5, 0.8);
        }
        else if(rings == Camera.Ring.ONE){
            drive.driveToPos(-8, 0.8);
            drive.turn(90, 0.8);

            wobble.rotateDown();
            sleep(500);
            wobble.open();

            drive.strafePID(0.8, 1);
        }
        else if(rings == Camera.Ring.FOUR){
            drive.driveToPos(-19, 0.8);

            wobble.rotateDown();
            sleep(500);
            wobble.open();

            drive.driveToPos(22, 0.8);
        }

        wobble.rotateUp();
        wobble.rotateUp();

        camera.shutdown();


    }


}
