package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.stuyfission.fissionlib.util.Mechanism;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CarriageSensor extends Mechanism {
    private DistanceSensor sensor;

    public CarriageSensor(LinearOpMode opMode) {this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) { sensor = hwMap.get(DistanceSensor.class, "distanceSensor"); }

    public double distanceCM() {
        return sensor.getDistance(DistanceUnit.CM);
    }
    public double distanceMM() {
        return sensor.getDistance(DistanceUnit.MM);
    }
}
