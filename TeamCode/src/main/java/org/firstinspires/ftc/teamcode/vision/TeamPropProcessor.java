package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TeamPropProcessor implements VisionProcessor {
    public enum Detection {
        LEFT,
        CENTER,
        RIGHT,
    }

    // with 640 by 480, tlLeft for the left third should be (0, 0), brLeft should be (213, 480), tlCenter should be (213, 0), brCenter should be (426, 480), tlRight should be (426, 0), brRight should be (640, 480). of course, these should be tuned for the actual prop and camera position.

    private Point tlLeft = new Point(0, 0);
    private Point brLeft = new Point(213, 480);

    private Point tlCenter = new Point(213, 0);
    private Point brCenter = new Point(426, 480);

    private Point tlRight = new Point(426, 0);
    private Point brRight = new Point(640, 480);

    private Scalar lowerThreshold = new Scalar(0, 0, 0);
    private Scalar upperThreshold = new Scalar(0, 0, 0);

    private Detection detection = null;
    private Object detectionLock = new Object();

    private Mat hsv = new Mat();
    private Mat threshold = new Mat();

    public TeamPropProcessor() {
    }

    public TeamPropProcessor(Point tlLeft, Point brLeft, Point tlCenter, Point brCenter, Point tlRight, Point brRight) {
        this.tlLeft = tlLeft;
        this.brLeft = brLeft;
        this.tlCenter = tlCenter;
        this.brCenter = brCenter;
        this.tlRight = tlRight;
        this.brRight = brRight;
    }

    public TeamPropProcessor(Point tlLeft, Point brLeft, Point tlCenter, Point brCenter, Point tlRight, Point brRight, Scalar lowerThreshold, Scalar upperThreshold) {
        this.tlLeft = tlLeft;
        this.brLeft = brLeft;
        this.tlCenter = tlCenter;
        this.brCenter = brCenter;
        this.tlRight = tlRight;
        this.brRight = brRight;
        this.lowerThreshold = lowerThreshold;
        this.upperThreshold = upperThreshold;
    }

    public TeamPropProcessor(Scalar lowerThreshold, Scalar upperThreshold) {
        this.lowerThreshold = lowerThreshold;
        this.upperThreshold = upperThreshold;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // nothing to do here, just make sure our camera is 640x480
        if (width != 640 || height != 480) {
            throw new IllegalArgumentException("Camera resolution must be 640x480");
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Process: check left, center, and right for threshold. Returns with best detection.
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowerThreshold, upperThreshold, threshold);

        // Check left
        int leftCount = Core.countNonZero(threshold.submat((int) tlLeft.y, (int) brLeft.y, (int) tlLeft.x, (int) brLeft.x));
        int centerCount = Core.countNonZero(threshold.submat((int) tlCenter.y, (int) brCenter.y, (int) tlCenter.x, (int) brCenter.x));
        int rightCount = Core.countNonZero(threshold.submat((int) tlRight.y, (int) brRight.y, (int) tlRight.x, (int) brRight.x));

        if (leftCount > centerCount && leftCount > rightCount) {
            synchronized (detectionLock) {
                detection = Detection.LEFT;
            }
        } else if (centerCount > leftCount && centerCount > rightCount) {
            synchronized (detectionLock) {
                detection = Detection.CENTER;
            }
        } else if (rightCount > leftCount && rightCount > centerCount) {
            synchronized (detectionLock) {
                detection = Detection.RIGHT;
            }
        } else {
            synchronized (detectionLock) {
                detection = Detection.CENTER; // Default to center
            }
        }

        return null; // This object is passed into onDrawFrame, but we don't need it
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Draw bounding box around location with color based on detection

        Detection detection = getDetection();

        Point tl = null;
        Point br = null;
        Paint color = new Paint();

        switch (detection) {
            case LEFT:
                tl = tlLeft;
                br = brLeft;
                color.setColor(Color.RED);
                break;
            case CENTER:
                tl = tlCenter;
                br = brCenter;
                color.setColor(Color.GREEN);
                break;
            case RIGHT:
                tl = tlRight;
                br = brRight;
                color.setColor(Color.BLUE);
                break;
            default:
                return;
        }

        canvas.drawRect(
                (float) tl.x * scaleBmpPxToCanvasPx,
                (float) tl.y * scaleBmpPxToCanvasPx,
                (float) br.x * scaleBmpPxToCanvasPx,
                (float) br.y * scaleBmpPxToCanvasPx,
                color
        );

        // Debug option: draw all bounding boxes

//        canvas.drawRect(
//                (float) tlLeft.x * scaleBmpPxToCanvasPx,
//                (float) tlLeft.y * scaleBmpPxToCanvasPx,
//                (float) brLeft.x * scaleBmpPxToCanvasPx,
//                (float) brLeft.y * scaleBmpPxToCanvasPx,
//                new Paint(Color.RED)
//        );
//
//        canvas.drawRect(
//                (float) tlCenter.x * scaleBmpPxToCanvasPx,
//                (float) tlCenter.y * scaleBmpPxToCanvasPx,
//                (float) brCenter.x * scaleBmpPxToCanvasPx,
//                (float) brCenter.y * scaleBmpPxToCanvasPx,
//                new Paint(Color.GREEN)
//        );
//
//        canvas.drawRect(
//                (float) tlRight.x * scaleBmpPxToCanvasPx,
//                (float) tlRight.y * scaleBmpPxToCanvasPx,
//                (float) brRight.x * scaleBmpPxToCanvasPx,
//                (float) brRight.y * scaleBmpPxToCanvasPx,
//                new Paint(Color.BLUE)
//        );
    }

    public Detection getDetection() {
        synchronized (detectionLock) {
            return detection;
        }
    }
}
