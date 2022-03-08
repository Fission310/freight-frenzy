package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Drivetrain extends Mechanism {
    public SampleMecanumDrive rrDrive;

    public enum DriveMode{
        ROBOT_CENTRIC,
        FIELD_CENTRIC_RED,
        FIELD_CENTRIC_BLUE,
    }
    DriveMode driveMode;

    public Drivetrain(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        rrDrive = new SampleMecanumDrive(hwMap);
        driveMode = DriveMode.ROBOT_CENTRIC;
    }

    public void strafeLeft(){
        rrDrive.setWeightedDrivePower(
                new Pose2d(
                        -1,
                        0,
                        0
                )
        );
    }

    public void strafeRight(){
        rrDrive.setWeightedDrivePower(
                new Pose2d(
                        1,
                        0,
                        0
                )
        );
    }

    public void loop(Gamepad gamepad){

        Pose2d poseEstimate = rrDrive.getPoseEstimate();
        Vector2d input;

        switch(driveMode){
            case ROBOT_CENTRIC:
                if(gamepad.dpad_left){
                    strafeLeft();
                }
                else if(gamepad.dpad_right){
                    strafeRight();
                }
                else{
                    rrDrive.setWeightedDrivePower(
                            new Pose2d(
                                    gamepad.left_stick_x,
                                    -gamepad.left_stick_y,
                                    -gamepad.right_stick_x
                            )
                    );
                }

                break;
            case FIELD_CENTRIC_BLUE:

                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                input = new Vector2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x
                ).rotated(-poseEstimate.getHeading() - Math.toRadians(90));

                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
                rrDrive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad.right_stick_x
                        )
                );

            case FIELD_CENTRIC_RED:
                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                input = new Vector2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x
                ).rotated(-poseEstimate.getHeading() + Math.toRadians(90));

                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
                rrDrive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad.right_stick_x
                        )
                );

        }

    }
}
