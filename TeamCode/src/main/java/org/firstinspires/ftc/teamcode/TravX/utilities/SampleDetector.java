package org.firstinspires.ftc.teamcode.TravX.utilities;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

public class SampleDetector extends OpenCvPipeline {

    // Parameters for smoothing
    private static final double ALPHA = 0.7; // Stronger smoothing to reduce oscillations

    // Parameters for deadband
    private static final double ANGLE_DEADBAND = 3.0;        // Minimum angle change to update

    // Parameters for choosing a larger object if it's not much farther
    private static final double DISTANCE_THRESHOLD = 100.0;  // Max additional distance allowed for a larger object
    private static final double AREA_RATIO = 2.0;            // How much larger the area must be

    private Point refPoint;
    private Point prevCenter = null;
    private double prevAngle = 0.0;

    // Output values
    private int relDx;
    private int relDy;
    private double chosenAngle;

    // Color ranges
    private Scalar lowerBound = new Scalar(5, 60, 30); // Default yellow
    private Scalar upperBound = new Scalar(35, 255, 255);
    private final Scalar lowerBound2 = new Scalar(170, 100, 100);                   // Additional range for red (if needed)
    private final Scalar upperBound2 = new Scalar(190, 255, 255);
    private boolean isRed = false;
    private boolean detectBothRedAndBlue = false;

    // Mats for processing
    private final Mat displayMat = new Mat();
    private final Mat hsvMat = new Mat();
    private final Mat maskMat = new Mat();
    private final Mat lightMask = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        int height = input.rows();
        int width = input.cols();
        refPoint = new Point((double) width / 2, height - 105);

        // Define the no-detection rectangle
        int rectWidth = width / 2;
        int rectHeight = 65; // Adjust height as needed
        Point noDetectTopLeft = new Point(refPoint.x - rectWidth / 1.5, height - rectHeight);
        Point noDetectBottomRight = new Point(refPoint.x + rectWidth / 1.5, height);

        // Convert to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold to get color mask
        if (detectBothRedAndBlue) {
            // Combine two ranges for red and blue
            Mat maskRed1 = new Mat();
            Mat maskRed2 = new Mat();
            Mat maskBlue = new Mat();

            // Red range 1 (lower red)
            Core.inRange(hsvMat, new Scalar(0, 100, 100), new Scalar(13, 255, 255), maskRed1);
            // Red range 2 (upper red)
            Core.inRange(hsvMat, lowerBound2, upperBound2, maskRed2);
            // Blue range
            Core.inRange(hsvMat, new Scalar(100, 60, 30), new Scalar(125, 255, 255), maskBlue);

            // Combine all masks (red and blue)
            Core.bitwise_or(maskRed1, maskRed2, maskMat);  // Combine the two red ranges
            Core.bitwise_or(maskMat, maskBlue, maskMat);    // Combine with the blue range

            maskRed1.release();
            maskRed2.release();
            maskBlue.release();
        } else if (isRed) {
            // Single range for red
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();

            Core.inRange(hsvMat, lowerBound, upperBound, mask1);
            Core.inRange(hsvMat, lowerBound2, upperBound2, mask2);

            Core.bitwise_or(mask1, mask2, maskMat);

            mask1.release();
            mask2.release();
        } else {
            // Single range for other colors (yellow/blue as set in `setSamplePickupColorIndex`)
            Core.inRange(hsvMat, lowerBound, upperBound, maskMat);
        }

        // Morphological operations
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4, 4));
        Imgproc.erode(maskMat, maskMat, kernel);
        Imgproc.dilate(maskMat, maskMat, kernel);

        // Convert to grayscale and filter dark areas
        Imgproc.cvtColor(input, lightMask, Imgproc.COLOR_RGB2GRAY);
        Imgproc.threshold(lightMask, lightMask, 30, 255, Imgproc.THRESH_BINARY);

        // Combine the HSV and light masks
        Core.bitwise_and(maskMat, lightMask, maskMat);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(maskMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.cvtColor(maskMat, displayMat, Imgproc.COLOR_GRAY2BGR); // Convert to BGR for colored drawing

        // Process candidates
        List<Candidate> candidates = new ArrayList<>();
        for (MatOfPoint cnt : contours) {
            double area = Imgproc.contourArea(cnt);
            if (area > 500) { // Adjust contour area threshold for noise filtering
                MatOfPoint2f contour2f = new MatOfPoint2f(cnt.toArray());
                RotatedRect rect = Imgproc.minAreaRect(contour2f);

                if (rect.center.x > noDetectTopLeft.x && rect.center.x < noDetectBottomRight.x &&
                        rect.center.y > noDetectTopLeft.y && rect.center.y < noDetectBottomRight.y) {
                    continue; // Skip this contour
                }

                double rawAngle = rect.angle;
                double angle = (rect.size.width < rect.size.height) ? rawAngle + 90.0 : rawAngle;

                double dx = rect.center.x - refPoint.x;
                double dy = rect.center.y - refPoint.y;
                double dist = Math.sqrt(dx * dx + dy * dy);

                candidates.add(new Candidate(dist, area, rect.center, rect, angle));
            }
        }

        // Choose the best candidate
        if (!candidates.isEmpty()) {
            candidates.sort(Comparator.comparingDouble(c -> c.dist));
            Candidate bestCandidate = candidates.get(0);

            for (Candidate candidate : candidates) {
                if ((candidate.dist - bestCandidate.dist) < DISTANCE_THRESHOLD && candidate.area > bestCandidate.area * AREA_RATIO) {
                    bestCandidate = candidate;
                }
            }

            // Smooth the center and angle
            if (prevCenter == null) {
                prevCenter = bestCandidate.center;
                prevAngle = bestCandidate.angle;
            } else {
                // Apply distance-based weighting for smoothing
                double distanceWeight = Math.min(1.0, bestCandidate.dist / 200.0); // Scale between 0 and 1
                double effectiveAlpha = ALPHA * distanceWeight;

                // Apply smoothing with a deadband
                double angleDiff = Math.abs(bestCandidate.angle - prevAngle);
                if (angleDiff > ANGLE_DEADBAND) { // Only update if change is significant
                    prevAngle = effectiveAlpha * bestCandidate.angle + (1 - effectiveAlpha) * prevAngle;
                }
                prevCenter.x = ALPHA * bestCandidate.center.x + (1 - ALPHA) * prevCenter.x;
                prevCenter.y = ALPHA * bestCandidate.center.y + (1 - ALPHA) * prevCenter.y;
            }

            // Set output values
            relDx = (int) (prevCenter.x - refPoint.x);
            relDy = (int) (prevCenter.y - refPoint.y);
            chosenAngle = prevAngle;

            // Draw the results
//            Imgproc.circle(input, refPoint, 5, new Scalar(0, 255, 0), 1);
//            Imgproc.circle(input, prevCenter, 5, new Scalar(0, 0, 255), 1);
//            Imgproc.line(input, refPoint, prevCenter, new Scalar(0, 255, 0), 2);
//            String infoText = String.format(Locale.US, "dx:%d, dy:%d, Angle:%.1fdeg", relDx, relDy, chosenAngle);
//            Imgproc.putText(input, infoText, new Point(refPoint.x + 10, refPoint.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);

            Imgproc.circle(displayMat, refPoint, 5, new Scalar(0, 255, 0), 1); // Green circle
            Imgproc.circle(displayMat, prevCenter, 5, new Scalar(0, 0, 255), 5); // Red circle
            Imgproc.line(displayMat, refPoint, prevCenter, new Scalar(0, 255, 0), 2); // Green line
            String infoText = String.format(Locale.US, "dx:%d, dy:%d, Angle:%.1fdeg", relDx, relDy, chosenAngle);
            Imgproc.putText(displayMat, infoText, new Point(refPoint.x + 10, refPoint.y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 0), 1);
            Imgproc.rectangle(displayMat, noDetectTopLeft, noDetectBottomRight, new Scalar(0, 255, 255), 2);

        } else {
            // No candidates detected, reset values
            relDx = 0;
            relDy = 0;
            chosenAngle = 0.0;

            // Optionally, clear previous center and angle
            prevCenter = null;
            prevAngle = 0.0;
        }
        return displayMat;
//        return input;
    }

    public int getRelDx() {
        return relDx;
    }

    public int getRelDy() {
        return relDy;
    }

    public double getChosenAngle() {
        return chosenAngle;
    }

    public void setSamplePickupColorIndex(int colorIndex) {
        if (colorIndex == 0) {
            // Yellow
            lowerBound = new Scalar(5, 60, 30);
            upperBound = new Scalar(35, 255, 255);
            isRed = false;
        } else if (colorIndex == 1) {
            // Blue
            lowerBound = new Scalar(100, 60, 30);
            upperBound = new Scalar(125, 255, 255);
            isRed = false;
        } else if (colorIndex == 2) {
            // Red
            lowerBound = new Scalar(0, 100, 100);
            upperBound = new Scalar(13, 255, 255);
            isRed = true;
        }
    }

    public void setDetectBothRedAndBlue(boolean detectBothRedAndBlue) {
        this.detectBothRedAndBlue = detectBothRedAndBlue;
    }

    private static class Candidate {
        double dist;
        double area;
        Point center;
        RotatedRect rect;
        double angle;

        Candidate(double dist, double area, Point center, RotatedRect rect, double angle) {
            this.dist = dist;
            this.area = area;
            this.center = center;
            this.rect = rect;
            this.angle = angle;
        }
    }
}
