package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Flicker;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Wobble;

@Autonomous(name="WobblePlace", group="Test")
public class WobblePlace extends LinearOpMode{


    private Drivetrain drive = new Drivetrain(this);
    private Acquirer acquirer = new Acquirer(this);
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
        acquirer.init(hardwareMap);

        waitForStart();

        drive.driveToPos(-2.25, 0.3);

        ElapsedTime time = new ElapsedTime();
        time.reset();
        while(time.seconds() < 4){
            rings = camera.listDetections();

            if(rings != Camera.Ring.NONE) break;
        }
        telemetry.addData("rings", rings);
        telemetry.update();

        drive.strafePID(-0.8, 0.72);
        drive.driveToPos(-20, 0.65);
        drive.turn(15,0.8);

        flywheel.forward();
        sleep(1000);
        for(int i = 0; i < 3; i++){
            flicker.swing();
            sleep(700);
            flicker.reset();
            sleep(700);
        }
        flywheel.stop();
        drive.turn(-15, 0.8);

        if(rings == Camera.Ring.NONE){

            drive.driveToPos(-5, 0.8);

            wobble.rotateDown();
            sleep(500);
            wobble.open();

            drive.strafePID(0.8, 0.3);

            drive.driveToPos(-2, 0.65);
        }
        else if(rings == Camera.Ring.ONE){
            drive.driveToPos(-14, 0.8);

            drive.strafePID(0.8, 0.7);

            wobble.rotateDown();
            sleep(500);
            wobble.open();

            drive.driveToPos(10, 0.8);
        }
        else if(rings == Camera.Ring.FOUR){
            drive.driveToPos(-25, 0.8);

            wobble.rotateDown();
            sleep(500);
            wobble.open();

            drive.driveToPos(12, 0.8);
//            acquirer.swing();
//            acquirer.forward();
//            drive.driveToPos(24, 0.8);
//
//            flywheel.forward();
//            sleep(1000);
//            for(int i = 0; i < 3; i++){
//                flicker.swing();
//                sleep(700);
//                flicker.reset();
//                sleep(700);
//            }
//            flywheel.stop();


        }

        wobble.rotateUp();
        wobble.rotateUp();

        camera.shutdown();


    }


}
