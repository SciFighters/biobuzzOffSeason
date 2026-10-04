package org.firstinspires.ftc.teamcode.subSystems;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import android.graphics.Canvas;
import android.graphics.Color;
import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

@Config
public class IntakeCameraSubsystem {

    public static double minBlobSize = 1000;
    public static int yellowHueMin = 0;
    public static int yellowHueMax = 35; // <- ^: good values
    public static double fox = 792.4171351060593;
    public static double foy = 795.423827473491;
    public static double pollenDiameterInches = 2.8;
    public static double centerX = 400;
    public static double centerY = 300;

    double cameraHeightInch = 2;
    double cameraPitchDegrees = 60; // Positive below the horizon.


    private final Scalar yellowMin = new Scalar(yellowHueMin, 120, 80);
    private final Scalar yellowMax = new Scalar(yellowHueMax, 255, 255);

    private final VisionPortal portal;
    private final Map<ColorRange, ColorBlobLocatorProcessor> blobProcessors = new HashMap<>();
    private final Map<ColorRange, List<Rect>> blobBounds = new HashMap<>();
    private final Set<ColorBlobLocatorProcessor> initializedBlobProcessors = new HashSet<>();
    private CameraCalibration calibration;

    public IntakeCameraSubsystem(HardwareMap hm, VisionProcessor... processors) {
        portal = new VisionPortal.Builder()
                .setCamera(hm.get(WebcamName.class, "cam"))
                .setCameraResolution(new Size(800, 600)) // 480p
                .enableLiveView(true)
                .addProcessor(new VisionProcessor() {
                    @Override
                    public void init(int width, int height, CameraCalibration cameraCalibration) {
                        synchronized (IntakeCameraSubsystem.this) {
                            calibration = cameraCalibration;
                            initializedBlobProcessors.clear();
                        }
                    }

                    @Override
                    public Object processFrame(Mat frame, long captureTimeNanos) {
                        int min = Math.max(0, Math.min(179, yellowHueMin));
                        int max = Math.max(0, Math.min(179, yellowHueMax));
                        yellowMin.val[0] = Math.min(min, max);
                        yellowMax.val[0] = Math.max(min, max);
                        Map<ColorBlobLocatorProcessor, Object> drawContexts = new HashMap<>();
                        ColorBlobLocatorProcessor.BlobFilter areaFilter =
                                new ColorBlobLocatorProcessor.BlobFilter(
                                        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                                        minBlobSize, Double.MAX_VALUE);
                        Map<ColorRange, ColorBlobLocatorProcessor> processors;
                        synchronized (IntakeCameraSubsystem.this) {
                            processors = new HashMap<>(blobProcessors);
                            for (ColorBlobLocatorProcessor processor : processors.values()) {
                                if (initializedBlobProcessors.add(processor)) {
                                    processor.init(frame.cols(), frame.rows(), calibration);
                                }
                            }
                        }
                        // Keep image processing off the lock used by the robot control loop.
                        for (Map.Entry<ColorRange, ColorBlobLocatorProcessor> entry : processors.entrySet()) {
                            ColorBlobLocatorProcessor processor = entry.getValue();
                            processor.removeAllFilters();
                            processor.addFilter(areaFilter);
                            drawContexts.put(processor, processor.processFrame(frame, captureTimeNanos));
                            List<Rect> bounds = new ArrayList<>();
                            for (ColorBlobLocatorProcessor.Blob blob : processor.getBlobs()) {
                                bounds.add(Imgproc.boundingRect(blob.getContour()));
                            }
                            synchronized (IntakeCameraSubsystem.this) {
                                blobBounds.put(entry.getKey(), bounds);
                            }
                        }
                        return drawContexts;
                    }

                    @Override
                    @SuppressWarnings("unchecked")
                    public void onDrawFrame(Canvas canvas, int width, int height,
                                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                            Object userContext) {
                        Map<ColorBlobLocatorProcessor, Object> drawContexts =
                                (Map<ColorBlobLocatorProcessor, Object>) userContext;
                        for (Map.Entry<ColorBlobLocatorProcessor, Object> entry : drawContexts.entrySet()) {
                            entry.getKey().onDrawFrame(canvas, width, height,
                                    scaleBmpPxToCanvasPx, scaleCanvasDensity, entry.getValue());
                        }
                    }
                })
                .addProcessors(processors)
                .build();
        FtcDashboard.getInstance().startCameraStream(portal, 10);
    }

    public void startStreaming() {
        portal.resumeStreaming();
        FtcDashboard.getInstance().startCameraStream(portal, 10);
    }

    public void stopStreaming() {
        FtcDashboard.getInstance().stopCameraStream();
        portal.stopStreaming();
    }

    public void setProcessorEnabled(VisionProcessor processor, boolean enabled) {
        portal.setProcessorEnabled(processor, enabled);
    }

    public VisionPortal.CameraState getCameraState() {
        return portal.getCameraState();
    }

    public double getFps() {
        return portal.getFps();
    }

    public void close() {
        FtcDashboard.getInstance().stopCameraStream();
        portal.close();
    }

    // Returns bounding rectangles of blobs of the given color.
    public synchronized List<Rect> scanBlobs(ColorRange color) {
        blobProcessors.computeIfAbsent(color, range -> {
            int overlayColor = range == ColorRange.RED ? Color.RED
                    : range == ColorRange.BLUE ? Color.BLUE
                      : range == ColorRange.YELLOW ? Color.YELLOW : Color.WHITE;
            return new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(range == ColorRange.YELLOW
                            ? new ColorRange(ColorSpace.HSV, yellowMin, yellowMax) : range)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame())
                    .setDrawContours(true)
                    .setContourColor(overlayColor)
                    .setBoxFitColor(overlayColor)
                    .build();
        });

        List<Rect> bounds = new ArrayList<>();
        for (Rect bound : blobBounds.getOrDefault(color, Collections.emptyList())) {
            bounds.add(bound.clone());
        }
        return bounds;
    }

    public synchronized List<Rect> scanBlobBounds(ColorRange color) {
        scanBlobs(color);
        ColorBlobLocatorProcessor processor = blobProcessors.get(color);
        List<Rect> bounds = new ArrayList<>();
        for (ColorBlobLocatorProcessor.Blob blob : processor.getBlobs()) {
            Rect bound = Imgproc.boundingRect(blob.getContour());
            bounds.add(bound);
        }
        return bounds;
    }

    // Estimates horizontal camera-to-blob distance in inches using the known pollen diameter.
    private double getBlobDistance(Rect blob) {
        double left = (centerX - blob.x - blob.width / 2.0) / fox;
        double up = (centerY - blob.y - blob.height / 2.0) / foy;
        double depth = pollenDiameterInches / 2 * (
                fox * Math.hypot(1, left) / blob.width + foy * Math.hypot(1, up) / blob.height);
        double pitch = Math.toRadians(cameraPitchDegrees);
        double forward = Math.cos(pitch) + up * Math.sin(pitch);
        return depth * Math.hypot(forward, left);
    }

    // Returns pitch-corrected robot-relative yaw; positive means left.
    public double getXAngleOffset(Rect blob) {
        double left = (centerX - blob.x - blob.width / 2.0) / fox;
        double up = (centerY - blob.y - blob.height / 2.0) / foy;
        double pitch = Math.toRadians(cameraPitchDegrees);
        double forward = Math.cos(pitch) + up * Math.sin(pitch);
        return Math.atan2(left, forward);
    }

    public double getYAngleOffset(Rect blob) {
        double left = (centerX - blob.x - blob.width / 2.0) / fox;
        double up = (centerY - blob.y - blob.height / 2.0) / foy;
        double pitch = Math.toRadians(cameraPitchDegrees);
        double forward = Math.cos(pitch) + up * Math.sin(pitch);
        double vertical = up * Math.cos(pitch) - Math.sin(pitch);
        return Math.atan2(vertical, Math.hypot(forward, left));
    }

    public double[] getElementLocation(Follower follower, Rect blob) {
        Pose base = follower.pose();

        double distanceInches = getBlobDistance(blob);
        double elementYaw = getXAngleOffset(blob);
        double heading = base.heading();

        double xOffSet = Math.cos(heading + elementYaw) * distanceInches;
        double yOffSet = Math.sin(heading + elementYaw) * distanceInches;
        return new double[] {base.x() + xOffSet, base.y() + yOffSet};
    }
}

public Path getElementsPath(Follower follower, ColorRange color) {
    List<Path> segments = new ArrayList<>();
    Pose start = follower.pose();
    for (Rect blob : scanBlobs(color)) {
        double[] location = getElementLocation(follower, blob);
        double dx = location[0] - start.x();
        double dy = location[1] - start.y();

        Pose target = new Pose(location[0], location[1], Math.atan2(dy, dx));
        segments.add(line(start, target).linear(start.heading(), target.heading()));
        start = target;
    }
    return path(segments.toArray(new Path[0]));
}

