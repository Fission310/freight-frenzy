package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Drivetrain extends SampleMecanumDrive {

    public enum DriveMode{
        ROBOT_CENTRIC,
        FIELD_CENTRIC_RED,
        FIELD_CENTRIC_BLUE,
    }
    DriveMode driveMode;

    public Drivetrain(HardwareMap hardwareMap) {
        super(hardwareMap);

        driveMode = DriveMode.ROBOT_CENTRIC;
    }

    public void strafeLeft(){
        setWeightedDrivePower(
                new Pose2d(
                        0,
                        -1,
                        0
                )
        );
    }

    public void strafeRight(){
        setWeightedDrivePower(
                new Pose2d(
                        0,
                        1,
                        0
                )
        );
    }

    public void loop(Gamepad gamepad){

        Pose2d poseEstimate = getPoseEstimate();
        Vector2d input;

        switch(driveMode){
            case ROBOT_CENTRIC:
                setWeightedDrivePower(
                        new Pose2d(
                                -gamepad.left_stick_y,
                                -gamepad.left_stick_x,
                                -gamepad.right_stick_x
                        )
                );
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
                setWeightedDrivePower(
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
                setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad.right_stick_x
                        )
                );

        }

    }
}
