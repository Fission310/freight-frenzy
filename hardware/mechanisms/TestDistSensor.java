package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class TestDistSensor extends Mechanism {
    DistanceSensor distanceSensor;

    public TestDistSensor(LinearOpMode opMode) { this.opMode = opMode; }

    // higher gain values -> smoother graph (more averaged values) but
    // significantly higher lag
    public static double HIGH_GAIN = 0.9;
    public static double LOW_GAIN = 0.2;
    LowPassFilter highFilter = new LowPassFilter(HIGH_GAIN);
    LowPassFilter lowFilter = new LowPassFilter(LOW_GAIN);

    @Override
    public void init(HardwareMap hwMap) {
        distanceSensor = hwMap.get(DistanceSensor.class, "dist");
    }

    public double getCM() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
    public double getFilteredHighCM() {
        double currentvalHigh = distanceSensor.getDistance(DistanceUnit.CM);
        double estimateHigh = highFilter.estimate(currentvalHigh);
        return estimateHigh;
    }
    public double getFilteredLowCM() {
        double currentvalLow = distanceSensor.getDistance(DistanceUnit.CM);
        double estimateLow = lowFilter.estimate(currentvalLow);
        return estimateLow;
    }
    public double getMM() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }
    public double getIN() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("IN", getIN());
        telemetry.addData("CM", getCM());
        telemetry.addData("Filtered High CM", getFilteredHighCM());
        telemetry.addData("Filtered Low CM", getFilteredLowCM());
        telemetry.addData("MM", getMM());
    }
}
