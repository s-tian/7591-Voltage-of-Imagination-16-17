package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VortexCenterPoint;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;
import org.firstinspires.ftc.teamcode.vision.LinearOpModeVision;
import org.lasarobotics.vision.detection.objects.Contour;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.opmodes.VisionShootingTest.VisionMode.PARTICLES;
import static org.firstinspires.ftc.teamcode.opmodes.VisionShootingTest.VisionMode.VORTEX;
import static org.firstinspires.ftc.teamcode.robotutil.Team.BLUE;
import static org.firstinspires.ftc.teamcode.robotutil.Team.RED;


/**
 * Created by Stephen on 12/23/2016.
 * Vision Shooting Test
 */

/*
    OpenCV Testing: Uses the LinearOpModeVision class that is supposed to gather image data, supposed to display the images from OpenCV to a camera view on the screen. Untested!!
 */

@SuppressWarnings("ConstantConditions")
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "VisionShootingTest", group = "Tests")

public class VisionShootingTest extends LinearOpModeVision {

    Mat mHsvMat;
    Mat mRgbaMat;
    Mat red1;
    Mat red2;
    Mat colorMask;
    Mat grayMask;
    Mat lowValMask;
    Mat lowSatMask;
    Mat blurred;
    Mat circles;
    MatOfInt convexHull;
    MatOfPoint2f temp2fMat;
    MatOfPoint2f polyApprox;
    MatOfInt4 convexityDefects;
    List<MatOfPoint> initialContourList;
    List<MatOfPoint> potentialContours;
    List<MatOfPoint> grayContours;
    List<Contour> resultContours;
    List<MatOfPoint> passedFirstCheck;
    VortexCenterPoint center;

    FlywheelTask flywheelTask;
    IntakeTask intakeTask;
    Servo phoneServo;

    boolean detectedVortex = false;

    int centX = -1;
    int centY = -1;

    static final int VORTEX_THRESHOLD = 3500;
    static final int IMAGE_HEIGHT = 600;       //ZTE Camera picture size
    static final int IMAGE_WIDTH = 800;
    static final double MAX_VORTEX_AREA_RATIO = 0.6;
    static final double psPosition = 0.2;

    static final int ACCEPTABLE_ERROR = 50;
    static final int PARTICLE_TARGET = 320;
    static final double MIN_PARTICLE_AREA_RATIO = 0.6;
    static final double PARTICLE_MIN_THRESHOLD = 500;
    static final double PARTICLE_MAX_THRESHOLD = 20000;
    double correctPower = 0.03;

    double[] blackArray = new double[] {0, 0, 0, 0};
    Scalar blackScalar = new Scalar(0, 0, 0);


    // this is for
    static int minVortexBlueH = 96;
    static int maxVortexBlueH = 120;
    static int minVortexBlueSat = 50;
    static int maxVortexBlueVal = 115;

    static int minBlueH = 80;
    static int maxBlueH = 110;
    static int minRedH = 160;
    static int maxRedH = 180;
    static int minBlueSat = 125;
    static int minRedSat = 50;
    static int minBlueVal = 75;
    static int minRedVal = 50;

    int cameraNumber = 0;
    MecanumDriveTrain driveTrain;
    boolean detectedTarget = false;
    ElapsedTime rejectTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int rejectTime = 500;

    Team team = BLUE;
    VisionMode visMode = PARTICLES;


    public enum VisionMode {
        VORTEX, PARTICLES
    }

    @Override
    public void runOpMode() {
        center = new VortexCenterPoint(-1, -1);
        initCamera(cameraNumber);   //Start OpenCV
        initVision();   //Do a bunch of initialization for vision code
        initRobot();
        options();
        waitForStart();
        startRobot();

        ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        if (cameraNumber == 1) {
            correctPower *= -1;
        }
        while(opModeIsActive()) {
            // too much to right is too big y, is negative power
            if (cameraNumber == 1) {
                correctPower = -0.03;
            } else {
                correctPower = 0.03;
            }
            rotateAim();
        }
        stopCamera();   //Tear down the camera instance
        System.out.println("Camera Stopped");
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {       //Callback from OpenCV leads here
        //Do image processing for individual frames here
        //Convert image to HSV format
        mRgbaMat = rgba;



        //Define two color ranges to match as red because the hue for red crosses over 180 to 0
        if (visMode == PARTICLES) {
            Imgproc.GaussianBlur(rgba, rgba, new Size(11, 11), 5, 5);
            Imgproc.cvtColor(rgba, mHsvMat, Imgproc.COLOR_RGB2HSV);
            return findCircles(rgba, gray);
        } else {
            Imgproc.GaussianBlur(rgba, rgba, new Size(5, 5), 2, 2);
            return findVortex(rgba, gray);
        }

    }

    public void initRobot() {
        //Initialize robot hardware and stuff here
        driveTrain = new MecanumDriveTrain(this);
        intakeTask = new IntakeTask(this);
        new CapBallTask(this);
        flywheelTask = new FlywheelTask(this);
        phoneServo = hardwareMap.servo.get("phoneServo");
    }

    public void initVision() {
        mHsvMat = new Mat();
        red1 = new Mat();
        red2 = new Mat();
        colorMask = new Mat();
        blurred = new Mat();
        convexHull = new MatOfInt();
        temp2fMat = new MatOfPoint2f();
        polyApprox = new MatOfPoint2f();
        convexityDefects = new MatOfInt4();
        initialContourList = new ArrayList<>();
        potentialContours = new ArrayList<>();
        passedFirstCheck = new ArrayList<>();
        resultContours = new ArrayList<>();
        grayContours = new ArrayList<>();
        grayMask = new Mat();
        lowValMask = new Mat();
        lowSatMask = new Mat();

    }
    public void rotateAim() {

        if (centY == -1) {
            //intakeTask.setPower(0);
            driveTrain.startRotation(0.07);
        } else if (centY < PARTICLE_TARGET - ACCEPTABLE_ERROR) {
            //intakeTask.setPower(0);
            driveTrain.startRotation(correctPower);
        } else if (centY > PARTICLE_TARGET + ACCEPTABLE_ERROR) {
            //intakeTask.setPower(0);
            driveTrain.startRotation(-correctPower);
        } else {
            driveTrain.stopAll();
            if (visMode == PARTICLES) {
                sleep(500);
                if (Math.abs(centY - PARTICLE_TARGET) < ACCEPTABLE_ERROR) {
                    driveForward();
                }
            }
        }
    }

    public void driveForward() {
        if (centX > IMAGE_WIDTH * 0.5) {
            intakeTask.setPower(1);
            driveTrain.powerAllMotors(0.2);
            rejectTimer.reset();
            while (opModeIsActive() && rejectTimer.time() < 3000) {
                if (intakeTask.correctColor()) {
                    driveTrain.stopAll();
                    changeVisMode(VORTEX);
                    return;
                }
            }
        } else {
            intakeTask.setPower(1);
            driveTrain.powerAllMotors(0.15);
        }
    }

    public void changeCamera(int a) {
        if (cameraNumber != a) {
            cameraNumber = a;
            stopCamera();
            initCamera(a);
            initVision();
        }
    }

    public void tileFilter(Mat rgba) {
        Mat lowMask = new Mat();
        Mat highMask = new Mat();
        Core.inRange(mHsvMat, new Scalar(0, 50, 50), new Scalar (50, 255, 255), lowMask);
        Core.inRange(mHsvMat, new Scalar (140, 50, 50), new Scalar(180, 255, 255), highMask);
        Core.inRange(mHsvMat, new Scalar(0, 175, 0), new Scalar(255, 255, 255), lowSatMask);
        Core.inRange(mHsvMat, new Scalar(0, 0, 175), new Scalar(255, 255, 255), lowValMask);
        Core.bitwise_or(lowSatMask, lowValMask, grayMask);
        Core.bitwise_or(lowMask, grayMask, grayMask);
        Core.bitwise_or(highMask, grayMask, grayMask);
        Imgproc.findContours(grayMask, grayContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < grayContours.size(); i ++) {
            MatOfPoint p = grayContours.get(i);
            Rect rect = Imgproc.boundingRect(p);
            int n = rect.x + rect.width/2;
            if (Imgproc.contourArea(p) > PARTICLE_MAX_THRESHOLD) {
                Imgproc.drawContours(rgba, grayContours, i, blackScalar, -1);
            }
        }
        Drawing.drawContours(rgba, resultContours, new ColorRGBA(0, 0, 255));

        resultContours.clear();
        grayContours.clear();
    }

    void options() {
        boolean confirmed = false;
        boolean upPressed = false;
        boolean downPressed = false;

        int modify = 0;
        int diff = 0;
        while (!confirmed) {

            if (gamepad1.dpad_up && !upPressed) {
                upPressed = true;
                diff = 2;
            }
            if (gamepad1.dpad_down && !downPressed) {
                downPressed = true;
                diff = -2;
            }
            if (!gamepad1.dpad_up) {
                upPressed = false;
            }
            if (!gamepad1.dpad_down) {
                downPressed = false;
            }
            if (gamepad1.y) {
                modify = 0;
            }
            if (gamepad1.b) {
                modify = 1;
            }
            if (gamepad1.a) {
                modify = 2;
            }
            if (gamepad1.x) {
                modify = 3;
            }
            if (gamepad1.right_bumper) {
                modify = 4;
            }
            if (gamepad1.left_bumper) {
                modify = 5;
            }
            if (gamepad1.right_trigger > 0.15) {
                modify = 6;
            }
            if (gamepad1.left_trigger > 0.15) {
                modify = 7;
            }
            if (gamepad1.right_stick_x < -0.15) {
                changeVisMode(VORTEX);
            }
            if (gamepad1.right_stick_x > 0.15) {
                changeVisMode(PARTICLES);
            }
            if (gamepad1.left_stick_x < -0.15) {
                team = RED;
            }
            if (gamepad1.left_stick_x > 0.15) {
                team = BLUE;
            }
            switch (modify) {
                case 0:
                    telemetry.addData("Modifying", "Min Blue");
                    minBlueH += diff;
                    diff = 0;
                    break;
                case 1:
                    telemetry.addData("Modifying", "Max Blue");
                    maxBlueH += diff;
                    diff = 0;
                    break;
                case 2:
                    telemetry.addData("Modifying", "Min Vortex Blue Hue");
                    minVortexBlueH += diff;
                    diff = 0;
                    break;
                case 3:
                    telemetry.addData("Modifying", "Max Vortex Blue Hue");
                    maxVortexBlueH += diff;
                    diff = 0;
                    break;
                case 4:
                    telemetry.addData("Modifying", "Min Blue Sat");
                    minBlueSat += diff;
                    diff = 0;
                    break;
                case 5:
                    telemetry.addData("Modifying", "Min Vortex Blue Sat");
                    minVortexBlueSat += diff;
                    diff = 0;
                    break;
                case 6:
                    telemetry.addData("Modifying", "Min Blue Val");
                    minBlueVal += diff;
                    diff = 0;
                    break;
                case 7:
                    telemetry.addData("Modifying", "Max Vortex Blue Val");
                    maxVortexBlueVal += diff;
                    diff = 0;
                    break;

            }

            telemetry.addData("Min Blue", minBlueH);
            telemetry.addData("Max Blue", maxBlueH);
            telemetry.addData("Min Vortex Blue H", minVortexBlueH);
            telemetry.addData("Max Vortex Blue H", maxVortexBlueH);
            telemetry.addData("Min Blue Sat", minBlueSat);
            telemetry.addData("Min Vortex Blue Sat", minVortexBlueSat);
            telemetry.addData("Min Blue Val", minBlueVal);
            telemetry.addData("Max Vortex Blue Val", maxVortexBlueVal);
            telemetry.addData("VisMode", visMode);
            telemetry.addData("Team", team);

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                confirmed = true;
                telemetry.addData("Confirmed!", "");
            }
            telemetry.update();

        }
    }

    boolean possibleBall(double area, int x) {
        if (cameraNumber == 1) {
            x = IMAGE_WIDTH - x;
        }
        return area > PARTICLE_MIN_THRESHOLD && area < Math.max(PARTICLE_MAX_THRESHOLD * x/IMAGE_WIDTH, 2000);
    }

    void calibrateTiles() {
        // Point phone at tiles so that only tiles are seen.
        long totalHue = 0;
        long totalSat = 0;
        long totalVal = 0;
        int totalPixels = mHsvMat.height() * mHsvMat.width();
        for (int i = 0; i < mHsvMat.height(); i++) {
            for (int j = 0; j < mHsvMat.width(); j++) {
                double[] colArr = mHsvMat.get(i,j);
                totalHue += colArr[0];
                totalSat += colArr[1];
                totalVal += colArr[2];
            }
        }
    }

    void changeVisMode (VisionMode visMode) {
        if (visMode != this.visMode) {
            this.visMode = visMode;
            if (visMode == PARTICLES) {
                changeCamera(0);
            } else {
                changeCamera(1);
            }
        }
    }

    void startRobot() {
        intakeTask.start();
        flywheelTask.start();
        phoneServo.setPosition(psPosition);
    }

    Mat findCircles(Mat rgba, Mat gray) {
        List<Mat> channels = new ArrayList<>();
        Core.split(mHsvMat, channels);
        Mat hue = channels.get(0);
        Mat sat = channels.get(1);
        Mat val = channels.get(2);
        Mat filtered = new Mat();
        if (team == BLUE) {
            Core.inRange(hue, new Scalar(minBlueH), new Scalar(maxBlueH), hue);
            Core.inRange(sat, new Scalar(minBlueSat), new Scalar(255), sat);
            Core.inRange(val, new Scalar(minBlueVal), new Scalar(255), val);
        } else if (team == RED) {
            Core.inRange(hue, new Scalar(minRedH), new Scalar(maxRedH), hue);
            Core.inRange(sat, new Scalar(minRedSat), new Scalar(255), sat);
            Core.inRange(val, new Scalar(minRedVal), new Scalar(255), val);
        }
        Core.bitwise_and(hue, sat, filtered);
        Core.bitwise_and(val, filtered, filtered);

        Imgproc.cvtColor(filtered.clone(), rgba, Imgproc.COLOR_GRAY2RGB);
        Imgproc.findContours(filtered.clone(), initialContourList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double minCircleArea = PARTICLE_MIN_THRESHOLD;
        for (MatOfPoint p : initialContourList) {
            double contourArea = Imgproc.contourArea(p);
            Rect rect = Imgproc.boundingRect(p);
            double radius = (rect.width + rect.height)/4;
            double circleArea = Math.PI * radius * radius;
            int centerX = rect.x + rect.width/2;
            int centerY = rect.y + rect.height/2;

            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 0, 255));
            if (centX != -1) {
                if (Math.abs(centerX - centX) > 100 || Math.abs(centerY - centY) > 100) {
                    continue;
                }
            }
            if ((double)rect.width/rect.height > 1.3 || (double)rect.height/rect.width > 1.3) {
                //continue;
            }

            if (contourArea/circleArea > 0.7) {
                Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 255, 0));
                Drawing.drawCircle(rgba, new Point(rect.x + rect.width/2, rect.y + rect.height/2), (int)(radius),new ColorRGBA(255, 0, 0) );
                if (contourArea > minCircleArea && possibleBall(contourArea, centerX)) {
                    minCircleArea = contourArea;
                    centX = centerX;
                    centY = centerY;
                    passedFirstCheck.clear();
                    passedFirstCheck.add(p);
                }
            }
        }
        if (passedFirstCheck.isEmpty()) {
            centX = -1;
            centY = -1;
        } else {
            Drawing.drawCircle(rgba,new Point(centX, centY), 10, new ColorRGBA(0, 255, 0));
        }
        initialContourList.clear();
        passedFirstCheck.clear();
        return rgba;
    }

    Mat objectDetection(Mat rgba, Mat gray) {
        List<Mat> channels = new ArrayList<>();
        Core.split(mHsvMat, channels);
        Mat hue = channels.get(0);
        Mat sat = channels.get(1);
        Mat val = channels.get(2);
        Mat filtered = new Mat();
        if (team == BLUE) {
            Core.inRange(hue, new Scalar(minVortexBlueH), new Scalar(maxVortexBlueH), hue);
            Core.inRange(sat, new Scalar(minVortexBlueSat), new Scalar(255), sat);
            Core.inRange(val, new Scalar(0), new Scalar(maxVortexBlueVal), val);
        } else if (team == RED) {
            Core.inRange(hue, new Scalar(minRedH), new Scalar(maxRedH), hue);
            Core.inRange(sat, new Scalar(minRedSat), new Scalar(255), sat);
            Core.inRange(val, new Scalar(minRedVal), new Scalar(255), val);
        }
        Core.bitwise_and(hue.clone(), sat.clone(), filtered);
        Core.bitwise_and(val.clone(), filtered.clone(), filtered);
        Imgproc.cvtColor(filtered, rgba, Imgproc.COLOR_GRAY2RGBA);

        initialContourList.clear();
        potentialContours.clear();
        passedFirstCheck.clear();
        resultContours.clear();
        //Find the external contours for the red mask using the fast simple approximation
        Imgproc.findContours(filtered.clone(), initialContourList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double minArea = VORTEX_THRESHOLD;
        for(MatOfPoint p: initialContourList) {        //Go through preliminary list of contours
            Rect rect = Imgproc.boundingRect(p);
            int centerY = rect.y + rect.height / 2;
            int centerX = rect.x + rect.width / 2;
            double contourArea = Imgproc.contourArea(p);

            if (contourArea < 500) {
                continue;
            }
            if (Math.abs(centerX - centX) > 100 || Math.abs(centerY - centY) > 100) {
                if (detectedVortex) {
                    continue;
                }
            }
            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 0, 0));
            // center vortex is not very solid shape, so if actual area is greater than 40% of rect area then it can't be vortex
            if (contourArea < VORTEX_THRESHOLD) {
                continue;
            }
            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 255, 0));
            if (contourArea/rect.area() < MAX_VORTEX_AREA_RATIO) {
                if (contourArea > minArea) {
                    minArea = contourArea;
                    centX = centerX;
                    centY = centerY;
                    passedFirstCheck.clear();
                    passedFirstCheck.add(p);
                }
            }
        }

        if(passedFirstCheck.isEmpty()) {
            centX = -1;
            centY = -1;
            detectedVortex = false;
            detectedTarget = false;
        } else {
            MatOfPoint vor = passedFirstCheck.get(0);
            Imgproc.drawContours(rgba, passedFirstCheck, 0, new Scalar(255, 0, 0));
            detectedVortex = true;
            detectedTarget = true;
        }
        Point vortexCenter = new Point(center.getX(), center.getY());
        Drawing.drawCircle(rgba, vortexCenter, 10, new ColorRGBA(255, 255, 255));
        return rgba;
    }

    Mat findVortex(Mat rgba, Mat gray) {
        Imgproc.cvtColor(rgba.clone(), mHsvMat, Imgproc.COLOR_RGB2HSV);

        List<Mat> channels = new ArrayList<>();

        Core.split(mHsvMat.clone(), channels);
        Mat hue = channels.get(0);
        Mat sat = channels.get(1);
        Mat val = channels.get(2);
        Mat filtered = new Mat();
        if (team == BLUE) {
            Core.inRange(hue, new Scalar(minVortexBlueH), new Scalar(maxVortexBlueH), hue);
            Core.inRange(sat, new Scalar(minVortexBlueSat), new Scalar(255), sat);
            Core.inRange(val, new Scalar(0), new Scalar(maxVortexBlueVal), val);
        } else if (team == RED) {
            Core.inRange(hue, new Scalar(minRedH), new Scalar(maxRedH), hue);
            Core.inRange(sat, new Scalar(minRedSat), new Scalar(255), sat);
            Core.inRange(val, new Scalar(minRedVal), new Scalar(255), val);
        }
        Core.bitwise_and(hue.clone(), sat.clone(), filtered);
        Core.bitwise_and(val.clone(), filtered.clone(), filtered);

        if (team == RED) {
            Core.inRange(mHsvMat, new Scalar(0, 50, 50), new Scalar(10, 255, 255), red1);
            Core.inRange(mHsvMat, new Scalar(160, 50, 50), new Scalar(240, 255, 255), colorMask);
        } else if (team == Team.BLUE) {
            Core.inRange(mHsvMat, new Scalar(minVortexBlueH, minVortexBlueSat, 0), new Scalar(maxVortexBlueH, 255, maxVortexBlueH), colorMask);
        }
        initialContourList.clear();
        potentialContours.clear();
        passedFirstCheck.clear();
        resultContours.clear();
        //Find the external contours for the red mask using the fast simple approximation
        //Imgproc.GaussianBlur(filtered, filtered, new Size(5, 5), 3, 3);
        Imgproc.cvtColor(filtered.clone(), rgba, Imgproc.COLOR_GRAY2RGB);

        Imgproc.findContours(filtered.clone(), initialContourList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double minArea = 100;
        for(MatOfPoint p: initialContourList) {
            p.convertTo(temp2fMat, CvType.CV_32F);      //Convert MatOfPoint to MatofPoint2f to run approxPolyDP
            double perimeter = Imgproc.arcLength(temp2fMat, true);
            Rect rect = Imgproc.boundingRect(p);
            int centerX = rect.x + rect.width/2;
            int centerY = rect.y + rect.height/2;
            double contourArea = Imgproc.contourArea(p);
            if (contourArea / rect.area() > MAX_VORTEX_AREA_RATIO) {
                continue;
            }
            if (contourArea > minArea) {
                minArea = contourArea;
                passedFirstCheck.clear();
                passedFirstCheck.add(p);
                centX = centerX;
                centY = centerY;
            }
        }//Go through preliminary list of contours


        int leftX = IMAGE_WIDTH;
        int rightX = 0;
        int topY = IMAGE_HEIGHT;
        int bottomY = 0;

        for(MatOfPoint p: passedFirstCheck) {
            resultContours.add(new Contour(p));
            Rect gRect = Imgproc.boundingRect(p);
            if(gRect.x+gRect.width > rightX) {
                rightX = gRect.x+gRect.width;
            }
            if(gRect.x < leftX) {
                leftX = gRect.x;
            }
            if(gRect.y+gRect.height > bottomY) {
                bottomY = gRect.y+gRect.height;
            }
            if(gRect.y < topY) {
                topY = gRect.y;
            }
        }
        centX = (leftX + rightX)/2;
        centY = (bottomY + topY)/2;
        Drawing.drawContours(rgba, resultContours, new ColorRGBA(255, 0, 0), 2);
        Drawing.drawCircle(rgba, new Point(centX, centY), 10, new ColorRGBA(255, 255, 255));

        return rgba;

    }

}

