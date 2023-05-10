package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import org.opencv.core.Core;
import org.opencv.core.Size;
import org.opencv.core.Scalar;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.ArrayList;

public class SUB_IntakeCamera extends SubsystemBase {
    private double detectionTime;
    private int threshold;
    private boolean hasTarget;
    private Pose2d targetPose;
    private boolean threadStarted;

    private Thread camThread;
    private OpenCVThread cvThread;

    private enum Stage {
        INPUT,
        CONVERTED,
        PROCESSED,
        FINAL
    }

    private Stage selectedStage;
    private SendableChooser<Stage> stageChooser;

    private int camWidth = 320, camHeight = 240;

    public SUB_IntakeCamera() {
        cvThread = new OpenCVThread();
        camThread = new Thread(cvThread);

        detectionTime = 0;
        hasTarget = false;
        targetPose = new Pose2d();

        selectedStage = Stage.FINAL;
        threshold = 155;//175;

        threadStarted = false;
    }

    public void init() {
        stageChooser = new SendableChooser<Stage>();
        stageChooser.setDefaultOption("FINAL", Stage.FINAL);
        stageChooser.addOption("INPUT", Stage.INPUT);
        stageChooser.addOption("CONVERTED", Stage.CONVERTED);
        stageChooser.addOption("PROCESSED", Stage.PROCESSED);

        SmartDashboard.putData("CVStageSelector", stageChooser);

        /* Start CV thread */
        if (!threadStarted) {
            camThread.setDaemon(true);
            camThread.start();
            threadStarted = true;
        }

        SmartDashboard.putNumber("CVThreshold", threshold);
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public Pose2d getTargetPose() {
        if (!hasTarget) {
            return new Pose2d();
        }

        return targetPose;
    }

    public double getTargetTx() {
        return targetPose.getX();
    }

    public double getTargetTy() {
        return targetPose.getY();
    }

    public double getTxZero() {
        return (double)camWidth / 2.0;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putBoolean("CVHasTarget", hasTarget);
      if (hasTarget) {
        SmartDashboard.putString("CVTargetPose", targetPose.toString());
      }
      SmartDashboard.putNumber("detectionTime(sec)", detectionTime);

      selectedStage = stageChooser.getSelected();
      threshold = (int)SmartDashboard.getNumber("CVThreshold", 0.0);
    }

    /* THREAD FOR RUNNING CAMERASERVER AND OPENCV PROCESSING */
    class OpenCVThread implements Runnable {
        private int CB_CHAN_IDX = 1;
        private Mat dilateElement;
        private Mat input; /* Original image */
        private Mat convertedMat; /* Image after converting to CrCB and extracing a specific channel */
        private Mat processedMat; /* Image after running image processing */
        private Mat finalMat; /* Original image with contours drawn */

        private void convertFrameToCrCb(Mat in, Mat out) {
            Imgproc.cvtColor(in, out, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(out, out, CB_CHAN_IDX);
        }

        private void applyThreshold(Mat in, Mat out) {
            Imgproc.threshold(in, out, threshold, 255, Imgproc.THRESH_BINARY);
        }

        private void morphMask(Mat in, Mat out) {
            Imgproc.dilate(in, out, dilateElement);
        }

        private boolean isGoodContour(MatOfPoint cont) {
            double a = Imgproc.contourArea(cont);
            Rect box = Imgproc.boundingRect(cont);
            double r = (double)box.width / (double)box.height;

            return (a >= 1600 && a < 17000) &&
                   (r >= 0.7 && r < 1.45);
        }

        private List<MatOfPoint> filterContours(Mat img) {
            List<MatOfPoint> contours = new ArrayList<>();
            List<MatOfPoint> filtered = new ArrayList<>();
            Imgproc.findContours(processedMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint cont = contours.get(i);

                if (isGoodContour(cont)) {
                    filtered.add(cont);
                }
            }

            return filtered;
        }

        private MatOfPoint findBestContour(List<MatOfPoint> contours) {
            /* Return the contour with the highest area */
            MatOfPoint best = contours.get(0);
            double max_a = Imgproc.contourArea(best);

            for (int i = 1; i < contours.size(); i++) {
                MatOfPoint cont = contours.get(i);
                double a = Imgproc.contourArea(cont);

                if (a > max_a) {
                    best = cont;
                    max_a = a;
                }
            }

            return best;
        }

        private Mat selectOutputMat(Stage s) {
            switch(s) {
                case INPUT:
                    return input;
                case CONVERTED:
                    return convertedMat;
                case PROCESSED:
                    return processedMat;
                case FINAL:
                    return finalMat;
            }

            /* Default/unknown case */
            return finalMat;
        }

        public void run() {
            UsbCamera camera = CameraServer.startAutomaticCapture();
            camera.setResolution(camWidth, camHeight);

            CvSink cvSink = CameraServer.getVideo();
            // CvSource outputStream = CameraServer.putVideo("Rectangle", camWidth, camHeight);

            /*
             * NOTE: All calls to OpenCV library must be made below this line (after the CameraServer.getVideo() call).
             * This is because WPILib will not load the OpenCV library until then. Attempting to make a call to OpenCV
             * before getVideo() will result in a library link runtime crash.
             */
            dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
            input = new Mat();
            convertedMat = new Mat();
            processedMat = new Mat();

            while (!Thread.interrupted()) {
                try {
                    if (cvSink.grabFrame(input) == 0) {
                        /* Error grabbing frame, ignore */
                        System.out.println("AAA: ERROR GRABBING FRAMES");
                        continue;
                    }
                    double start = Timer.getFPGATimestamp();

                    /* Clone the original image to draw the contours on */
                    finalMat = input.clone();

                    convertFrameToCrCb(input, convertedMat);
                    applyThreshold(convertedMat, processedMat);

                    morphMask(processedMat, processedMat);
                    // morphMask(processedMat, processedMat);
                    // morphMask(processedMat, processedMat);
                    // morphMask(processedMat, processedMat);
                    // morphMask(processedMat, processedMat);

                    List<MatOfPoint> contours = filterContours(processedMat);

                    hasTarget = false;
                    if (!contours.isEmpty()) {
                        hasTarget = true;

                        MatOfPoint best = findBestContour(contours);

                        Rect box = Imgproc.boundingRect(best);
                        double a = Imgproc.contourArea(best);
                        double r = (double)box.width / (double)box.height;

                        Scalar color = new Scalar(0, 255, 0);
                        Imgproc.rectangle(finalMat, box, color);

                        targetPose = new Pose2d(box.x + (double)box.width / 2.0, box.y + (double)box.height / 2.0, new Rotation2d());

                        Imgproc.putText(finalMat, "Pose: " + targetPose, new Point(box.x, box.y - 10), Imgproc.FONT_HERSHEY_PLAIN, 1, color);
                        Imgproc.putText(finalMat, "Ratio: " + String.valueOf(r), new Point(box.x, box.y - 20), Imgproc.FONT_HERSHEY_PLAIN, 1, color);
                        Imgproc.putText(finalMat, "Area: " + String.valueOf(a), new Point(box.x, box.y - 30), Imgproc.FONT_HERSHEY_PLAIN, 1, color);

                        Imgproc.line(finalMat, new Point(targetPose.getX(), 0), new Point(targetPose.getX(), camHeight), new Scalar(0, 0, 255), 2);
                    }

                    detectionTime = Timer.getFPGATimestamp() - start;

                    // outputStream.putSFrame(selectOutputMat(selectedStage));

                    /* Sleep to avoid overruning the RoboRio */
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println("OpenCVThread Exception: " + e);
                }
            }
        }
    }
}
