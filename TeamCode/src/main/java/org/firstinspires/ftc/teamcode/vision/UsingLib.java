package org.firstinspires.ftc.teamcode.vision;

import org.lasarobotics.vision.opmode.ManualVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.opencv.core.Mat;

/**
 * Created by Stephen on 12/24/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "UsingLib", group = "Tests")

public class UsingLib extends ManualVisionOpMode {
    public Mat frame(Mat rgba, Mat gray) {
        return rgba;
    }
}
