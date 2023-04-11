package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import org.opencv.core.Mat;

public class SUB_IntakeCamera extends Thread {

    @Override
    public void run() {
        Mat mat = new Mat();

        while (!Thread.interrupted()) {
            UsbCamera camera = CameraServer.startAutomaticCapture();
            camera.setResolution(640, 480);

            CvSink cvSink = CameraServer.getVideo();
            CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

            if (cvSink.grabFrameNoTimeout(mat) == 0) {
                /* Error grabbing frame, ignore */
                continue;
            }

            outputStream.putFrame(mat);
        }
    }
}
