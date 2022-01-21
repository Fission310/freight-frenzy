package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// TODO: @Config, change to public static
public class BarcodeDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }
    private Location location;

    // Top left corner is 0,0
    // view is 1280 x 720
    static final int SCREEN_WIDTH = 1280;
    static final int SCREEN_HEIGHT = 720;
    static final int SIDE_LEN = 250;
    static final int CENTER_POS = (SCREEN_HEIGHT / 2) - (SIDE_LEN / 2);
    static final int NUM_ROI = 3;
    static final int GAP = (SCREEN_WIDTH - (SIDE_LEN * NUM_ROI)) / (NUM_ROI + 1);

    static final Rect LEFT_ROI = new Rect(
            new Point(GAP, CENTER_POS),
            new Point(GAP + SIDE_LEN, CENTER_POS + SIDE_LEN));
    static final Rect MIDDLE_ROI = new Rect(
            new Point((GAP * 2) + SIDE_LEN, CENTER_POS),
            new Point((GAP * 2) + (SIDE_LEN * 2), CENTER_POS + SIDE_LEN));
    static final Rect RIGHT_ROI = new Rect(
            new Point((GAP * 3) + (SIDE_LEN * 2), CENTER_POS),
            new Point((GAP * 3) + (SIDE_LEN * 3), CENTER_POS + SIDE_LEN));
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public BarcodeDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        middle.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean duckLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean duckMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean duckRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (duckLeft) {
            location = Location.LEFT;
            telemetry.addData("Duck Location", "left");
        }
        else if (duckMiddle) {
            location = Location.MIDDLE;
            telemetry.addData("Duck Location", "middle");
        }
        else if (duckRight) {
            location = Location.RIGHT;
            telemetry.addData("Duck Location", "right");
        } else {
            location = Location.NOT_FOUND;
            telemetry.addData("Duck Location", "not found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorFound = new Scalar(0, 255, 0);
        Scalar colorNotFound = new Scalar(255, 0, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorFound:colorNotFound);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? colorFound:colorNotFound);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorFound:colorNotFound);

        return mat;
    }

    public Location getLocation() {
        return location;
    }

}
