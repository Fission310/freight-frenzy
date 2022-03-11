package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.stuyfission.fissionlib.util.Mechanism;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class TestDistSensor extends Mechanism {
    DistanceSensor distanceSensor;

    public TestDistSensor(LinearOpMode opMode) { this.opMode = opMode; }

    // higher gain values -> smoother graph (more averaged values) but
    // significantly higher lag
    public static double GAIN = 0.2;
    LowPassFilter lowPassFilter = new LowPassFilter(GAIN);

    // Kalman filter
    public static double Q = 0.3;
    public static double R = 3;
    public static int N = 3;
    KalmanFilter kalmanFilter = new KalmanFilter(Q,R,N);

    @Override
    public void init(HardwareMap hwMap) {
        distanceSensor = hwMap.get(DistanceSensor.class, "dist");
    }

    public double getCM() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public double getLowPass() {
        double currentValue = distanceSensor.getDistance(DistanceUnit.CM);
        double estimate = lowPassFilter.estimate(currentValue);
        return estimate;
    }

    public double getKalman() {
        double currentValue = distanceSensor.getDistance(DistanceUnit.CM);  // imaginary, noisy sensor
        double estimate = kalmanFilter.estimate(currentValue); // smoothed sensor
        return estimate;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("CM", getCM());
        telemetry.addData("Low Pass", getLowPass());
        telemetry.addData("Kalman", getKalman());
        telemetry.addData("Low Pass Offset", Math.abs(getCM() - getLowPass()));
        telemetry.addData("Kalman Offset", Math.abs(getCM() - getKalman()));
    }
}
