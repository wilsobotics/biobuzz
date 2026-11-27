/*
 * Hive Vision — Control Hub track (OpenCV, no model)
 * Color-threshold ball detector for FTC. Runs on the Control Hub via
 * VisionPortal (VisionProcessor), no coprocessor required.
 *
 * HSV ranges and geometry gates were learned from the Limelight track's
 * YOLO model on real match footage (fit_hsv_from_yolo.py).
 * Re-tune on your actual field camera with hsv_tuner.py and copy the
 * constants here if lighting differs.
 *
 * Measured quality on the development match video (vs YOLO truth) is low -
 * roughly 26-65% precision - so treat these as candidates, and never act on a
 * single frame in autonomous: require the same colour to win several
 * consecutive frames before committing to it.
 *
 * Typical HSV (OpenCV H 0..180, S/V 0..255):
 *   yellow: H 16..33,   S 90..255,  V 45..238
 *   red   : H 0..15 and 170..180,  S 81..255,  V 45..232
 *   blue  : H 102..128, S 80..255, V 45..247
 *
 * Yellow starts at H 16 because the old H 9 lower bound overlapped red's 0..15
 * range, which let a single orange blob count as both yellow and red.
 */
package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

public class BallDetectorPipeline implements VisionProcessor {

    /* ------------------------------------------------------------------
     * 1) HSV segments (lower/upper per color). Red is matched with two
     *    segments because its hue wraps through 0.
     * ----------------------------------------------------------------*/
    public enum BallColor { YELLOW, RED, BLUE }

    private static final Scalar YELLOW_LO = new Scalar(16, 90, 45);
    private static final Scalar YELLOW_HI = new Scalar(33, 255, 238);

    private static final Scalar RED_LO_A = new Scalar(0, 81, 45);
    private static final Scalar RED_HI_A = new Scalar(15, 255, 232);
    private static final Scalar RED_LO_B = new Scalar(170, 81, 45);
    private static final Scalar RED_HI_B = new Scalar(180, 255, 232);

    private static final Scalar BLUE_LO = new Scalar(102, 80, 45);
    private static final Scalar BLUE_HI = new Scalar(128, 255, 247);

    /* ------------------------------------------------------------------
     * 2) Shape/size gating, expressed as a fraction of the frame area so the
     *    gates survive a change of camera resolution. The numbers in the
     *    comments are the pixel values they work out to on a 640x480 frame.
     *    Real balls on the FTC field are small-to-mid and roughly square;
     *    robot side panels are giant or elongated. Tune for your camera.
     * ----------------------------------------------------------------*/
    private static final int   REFERENCE_AREA_PX = 640 * 480;

    private static final double MIN_AREA_FRAC = 900.0 / REFERENCE_AREA_PX;    // kills speckle patches
    private static final double MAX_AREA_FRAC = 12000.0 / REFERENCE_AREA_PX;  // kills big robot panels
    private static final double MIN_ASPECT  = 0.71;   // near-square balls only
    private static final double MAX_ASPECT  = 1.4;
    private static final double MIN_FILL    = 0.5;    // contourArea / rectArea
    private static final boolean USE_MORPH = true;    // close small gaps

    /** Typical real-ball contour area per color, as a fraction of the frame. */
    private static final double EXPECT_YELLOW_FRAC = 3500.0 / REFERENCE_AREA_PX;
    private static final double EXPECT_RED_FRAC    = 4200.0 / REFERENCE_AREA_PX;
    private static final double EXPECT_BLUE_FRAC   = 4200.0 / REFERENCE_AREA_PX;

    private static double expectAreaFrac(BallColor c) {
        switch (c) {
            case YELLOW: return EXPECT_YELLOW_FRAC;
            case BLUE:   return EXPECT_BLUE_FRAC;
            default:     return EXPECT_RED_FRAC;
        }
    }

    private boolean suspend = false;
    private boolean detect  = true;
    private final Object lock = new Object();
    private List<BallBlob> detections = new ArrayList<>();

    private int frameAreaPx = REFERENCE_AREA_PX;
    private double minAreaPx = MIN_AREA_FRAC * REFERENCE_AREA_PX;
    private double maxAreaPx = MAX_AREA_FRAC * REFERENCE_AREA_PX;

    /** One detected ball candidate for the OpMode. */
    public static class BallBlob {
        public final BallColor color;
        public final Rect   rect;
        public final double area;
        public final double cx, cy;
        public double cxNorm = 0, cyNorm = 0, wNorm = 0, hNorm = 0;

        BallBlob(BallColor c, Rect r, double a) {
            color = c;
            rect = r;
            area = a;
            cx = r.x + r.width / 2.0;
            cy = r.y + r.height / 2.0;
        }

        void normalize(int frameW, int frameH) {
            cxNorm = cx / frameW;
            cyNorm = cy / frameH;
            wNorm = rect.width / (double) frameW;
            hNorm = rect.height / (double) frameH;
        }
    }

    /** Most ball-like detected blob of a color (candidate, not ground truth). */
    public BallBlob bestOf(BallColor color) {
        BallBlob best = null;
        double bestCost = Double.MAX_VALUE;
        double expected = expectAreaFrac(color) * frameAreaPx;
        synchronized (lock) {
            for (BallBlob b : detections) {
                if (b.color != color) continue;
                double fill = b.area / (double) (b.rect.width * b.rect.height);
                double cost = Math.abs(b.area / expected - 1.0)
                            + 4.0 * (1.0 - fill);
                if (cost < bestCost) {
                    bestCost = cost;
                    best = b;
                }
            }
        }
        return best;
    }

    public List<BallBlob> getDetections() {
        synchronized (lock) {
            return new ArrayList<>(detections);
        }
    }

    public void suspendDetection() { detect = false; }
    public void resumeDetection()  { detect = true; }
    public void suspend()          { suspend = true; }
    public void resume()           { suspend = false; }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Only used to scale the pixel-area gates to the camera's actual resolution.
        if (width > 0 && height > 0) {
            frameAreaPx = width * height;
            minAreaPx = MIN_AREA_FRAC * frameAreaPx;
            maxAreaPx = MAX_AREA_FRAC * frameAreaPx;
        }
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        if (suspend) {
            return input; // return unmodified
        }
        List<BallBlob> out = new ArrayList<>();
        if (detect) {
            maskAndFind(input, BallColor.YELLOW, YELLOW_LO, YELLOW_HI, null, null, out);
            maskAndFind(input, BallColor.RED,    RED_LO_A, RED_HI_A, RED_LO_B, RED_HI_B, out);
            maskAndFind(input, BallColor.BLUE,   BLUE_LO, BLUE_HI, null, null, out);
        }
        for (BallBlob b : out) b.normalize(input.cols(), input.rows());

        synchronized (lock) { detections = out; }

        // The return value only reaches onDrawFrame as userContext; VisionPortal does not
        // display it. Annotating the input frame in place is what makes the overlay show,
        // and it costs no per-frame native allocation.
        if (detect) drawOverlay(input, out);
        return input;
    }

    @Override
    public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Overlay is drawn onto the frame in processFrame; no custom canvas drawing needed.
    }

    private void maskAndFind(Mat input, BallColor color, Scalar loA, Scalar hiA,
                             Scalar loB, Scalar hiB, List<BallBlob> out) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        if (loB != null) { // red wrap: union of two masks
            Mat m2 = new Mat();
            Core.inRange(hsv, loA, hiA, mask);
            Core.inRange(hsv, loB, hiB, m2);
            Core.bitwise_or(mask, m2, mask);
            m2.release();
        } else {
            Core.inRange(hsv, loA, hiA, mask);
        }
        hsv.release();
        if (USE_MORPH) {
            Mat el = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                                                   new org.opencv.core.Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, el);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, el);
            el.release();
        }
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hier = new Mat();
        Imgproc.findContours(mask, contours, hier, Imgproc.RETR_EXTERNAL,
                             Imgproc.CHAIN_APPROX_SIMPLE);
        hier.release();
        mask.release();
        for (MatOfPoint ct : contours) {
            double area = Imgproc.contourArea(ct);
            Rect r = Imgproc.boundingRect(ct);
            ct.release();
            if (area < minAreaPx || area > maxAreaPx) continue;
            if (r.width <= 0 || r.height <= 0) continue;
            double aspect = (double) r.width / r.height;
            if (aspect < MIN_ASPECT || aspect > MAX_ASPECT) continue;
            double fill = area / (double) (r.width * r.height);
            if (fill < MIN_FILL) continue;
            out.add(new BallBlob(color, r, area));
        }
    }

    private void drawOverlay(Mat frame, List<BallBlob> blobs) {
        for (BallBlob b : blobs) {
            Scalar col = b.color == BallColor.YELLOW ? new Scalar(0, 255, 255)
                       : b.color == BallColor.RED    ? new Scalar(0, 0, 255)
                       : new Scalar(255, 0, 0);
            Imgproc.rectangle(frame, b.rect, col, 2);
            Imgproc.putText(frame, b.color.name(),
                            new Point(b.rect.x, b.rect.y - 6),
                            Imgproc.FONT_HERSHEY_PLAIN, 1.0, col, 2);
        }
    }
}