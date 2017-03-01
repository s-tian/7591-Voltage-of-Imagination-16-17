package org.firstinspires.ftc.teamcode.opmodes;


import android.util.Log;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VortexCenterPoint;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
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


    IntakeTask intakeTask;
    Servo phoneServo;

    boolean detectedVortex = false;

    int centX = -1;
    int centY = -1;

    static final int VORTEX_THRESHOLD = 3500;
    static final int IMAGE_HEIGHT = 600;       //ZTE Camera picture size
    static final int IMAGE_WIDTH = 800;
    static final double MAX_VORTEX_AREA_RATIO = 0.6;

    static final int ACCEPTABLE_ERROR = 50;
    static final int PARTICLE_TARGET = 320;
    static final double MIN_PARTICLE_AREA_RATIO = 0.6;
    static final double PARTICLE_MIN_THRESHOLD = 500;
    static final double PARTICLE_MAX_THRESHOLD = 15000;
    double correctPower = 0.03;

    double[] blackArray = new double[] {0, 0, 0, 0};
    Scalar blackScalar = new Scalar(0, 0, 0);


    static int minBlueH = 90;
    static int maxBlueH = 110;
    static int minRedH = 160;
    static int maxRedH = 180;
    static int minBlueSat = 125;
    static int minRedSat = 50;
    static int minBlueVal = 100;
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
        //options();
        intakeTask.start();
        waitForStart();

        ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        if (cameraNumber == 1) {
            correctPower *= -1;
        }
        String lastString = "";
        while(opModeIsActive()) {
            // too much to right is too big y, is negative power
            String colorString = intakeTask.voiColorIntake.getColor();
            if (!colorString.equals(lastString)) {
                System.out.println("COLOR FROM T: " + colorString);
                lastString = colorString;
            }
            if (intakeTask.voiColorIntake.wrongColor()) {
                System.out.println("Wrong Color! E");
                rejectTimer.reset();
                intakeTask.setPower(-1);
                while (opModeIsActive() && rejectTimer.time() < rejectTime);
                intakeTask.setPower(0);
                continue;
            }
            if (intakeTask.voiColorIntake.correctColor()) {
                sleep(500);
                while (intakeTask.voiColorIntake.correctColor() && opModeIsActive());
                intakeTask.setPower(0);
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

        Imgproc.GaussianBlur(rgba, rgba, new Size(11, 11), 5, 5);

        Imgproc.cvtColor(rgba, mHsvMat, Imgproc.COLOR_RGB2HSV);

        //Define two color ranges to match as red because the hue for red crosses over 180 to 0
        if (visMode == PARTICLES) {
            return findCircles(rgba, gray);
        } else {
            return rgba;
        }

    }

    public void initRobot() {
        //Initialize robot hardware and stuff here
        driveTrain = new MecanumDriveTrain(this);
        intakeTask = new IntakeTask(this);
        new CapBallTask(this);
        phoneServo = hardwareMap.servo.get("phoneServo");
        phoneServo.setPosition(0.2);
        sleep(1000);
        phoneServo.setPosition(0.2);
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
            driveTrain.startRotation(0.1);
        } else if (centY < PARTICLE_TARGET - ACCEPTABLE_ERROR) {
            //intakeTask.setPower(0);
            driveTrain.startRotation(correctPower);
        } else if (centY > PARTICLE_TARGET + ACCEPTABLE_ERROR) {
            //intakeTask.setPower(0);
            driveTrain.startRotation(-correctPower);
        } else {
            driveTrain.stopAll();
            sleep(500);
            if (Math.abs(centY - PARTICLE_TARGET) < ACCEPTABLE_ERROR) {
                driveForward();
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
                visMode = VORTEX;
            }
            if (gamepad1.right_stick_x > 0.15) {
                visMode = PARTICLES;
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
                    telemetry.addData("Modifying", "Min Red");
                    minRedH += diff;
                    diff = 0;
                    break;
                case 3:
                    telemetry.addData("Modifying", "Max Red");
                    maxRedH += diff;
                    diff = 0;
                    break;
                case 4:
                    telemetry.addData("Modifying", "Min Blue Sat");
                    minBlueSat += diff;
                    diff = 0;
                    break;
                case 5:
                    telemetry.addData("Modifying", "Min Red Sat");
                    minRedSat += diff;
                    diff = 0;
                    break;
                case 6:
                    telemetry.addData("Modifying", "Min Blue Val");
                    minBlueVal += diff;
                    diff = 0;
                    break;
                case 7:
                    telemetry.addData("Modifying", "Min Red Val");
                    minRedVal += diff;
                    diff = 0;
                    break;

            }

            telemetry.addData("Min Blue", minBlueH);
            telemetry.addData("Max Blue", maxBlueH);
            telemetry.addData("Min Red", minRedH);
            telemetry.addData("Max Red", maxRedH);
            telemetry.addData("Min Blue Sat", minBlueSat);
            telemetry.addData("Min Red Sat", minRedSat);
            telemetry.addData("Min Blue Val", minBlueVal);
            telemetry.addData("Min Red Val", minRedVal);
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
                continue;
            }

            if (contourArea/circleArea > 0.7) {
                Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 255, 0));
                Drawing.drawCircle(rgba, new Point(rect.x + rect.width/2, rect.y + rect.height/2), (int)(radius),new ColorRGBA(255, 0, 0) );
                if (contourArea > minCircleArea) {
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
        if (visMode == PARTICLES) {
            //tileFilter(rgba);
            //findCircles(rgba, gray);
        }


        if (team == RED) {
            //Core.inRange(mHsvMat, new Scalar(0, 50, 40), new Scalar(10, 255, 255), red1);
            Core.inRange(mHsvMat, new Scalar(minRedH, minRedSat, minRedVal), new Scalar(maxRedH, 255, 255), colorMask);
            //Core.bitwise_or(red1, red2, colorMask);

        } else if (team == BLUE) {
            if (visMode == VORTEX) {
                Core.inRange(mHsvMat, new Scalar(minBlueH, minBlueSat, minBlueVal), new Scalar(maxBlueH, 255, 255), colorMask);
            } else if (visMode == PARTICLES) {
                Core.inRange(mHsvMat, new Scalar(minBlueH, minBlueSat, minBlueVal), new Scalar(maxBlueH, 255, 255), colorMask);
            }
        }
        //OR the two masks together to produce a mask that combines the ranges
        //Core.addWeighted(red1, 1.0, red2, 1.0, 0.0, colorMask);

        initialContourList.clear();
        potentialContours.clear();
        passedFirstCheck.clear();
        resultContours.clear();
        //Find the external contours for the red mask using the fast simple approximation
        Imgproc.findContours(colorMask, initialContourList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for(MatOfPoint p: initialContourList) {        //Go through preliminary list of contours
            p.convertTo(temp2fMat, CvType.CV_32F);      //Convert MatOfPoint to MatofPoint2f to run approxPolyDP
            double perimeter = Imgproc.arcLength(temp2fMat, true);
            //Approximate each contour using a polygon
            Imgproc.approxPolyDP(temp2fMat, polyApprox, 0.02*perimeter, true);
            MatOfPoint polyApproxFloat = new MatOfPoint(polyApprox.toArray());
            Rect rect = Imgproc.boundingRect(polyApproxFloat);
            int y = rect.y + rect.height / 2;
            int x = rect.x + rect.width / 2;
            double contourArea = Imgproc.contourArea(p);

            if (contourArea < 500) {
                continue;
            }
            //Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 255, 255));

            if (Math.abs(x - center.getX()) > 100 || Math.abs(y - center.getY()) > 100) {
                if (detectedVortex) {
                    continue;
                }
            }



            if (visMode == VORTEX) {

                // center vortex is not very solid shape, so if actual area is greater than 40% of rect area then it can't be vortex
                if (contourArea < VORTEX_THRESHOLD) {
                    continue;
                }
                Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 255, 0));

                if (polyApproxFloat.toArray().length > 4 && rect.x < IMAGE_WIDTH / 2) {
                    if (contourArea/rect.area() < MAX_VORTEX_AREA_RATIO) {
                        Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 0, 255));
                        Imgproc.convexHull(polyApproxFloat, convexHull);
                        if (convexHull.rows() > 2) {
                            Imgproc.convexityDefects(polyApproxFloat, convexHull, convexityDefects);
                            List<Integer> cdlist = convexityDefects.toList();
                            int count = 0;
                            for (int i = 0; i < cdlist.size(); i += 4) {
                                double depth = cdlist.get(i + 3) / 256.0;
                                if (depth > Math.sqrt(rect.height) / 2) {
                                    count++;
                                }
                            }
                            if (count > 0) {
                                passedFirstCheck.add(p);
                            }
                        }

                    } else {
                        potentialContours.add(p);
                    }
                }
            } else if (visMode == PARTICLES) {
                // circle should fill up rectangle pretty well so if it doesn't then skip
                if (contourArea/rect.area() < MIN_PARTICLE_AREA_RATIO) {
                    continue;
                }
                Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 255, 0));

                // if correct size
                //System.out.println("Contour area: " + contourArea + " MAX: " + Math.max(PARTICLE_MAX_THRESHOLD * (IMAGE_WIDTH - x)/IMAGE_WIDTH, 2000));
                if (possibleBall(contourArea, x)) {
                    passedFirstCheck.add(p);
                    Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 0, 0));
                }

            }
        }
        // take only the largest contour
        while (passedFirstCheck.size() > 1) {
            double size0 = Imgproc.contourArea(passedFirstCheck.get(0));
            double size1 = Imgproc.contourArea(passedFirstCheck.get(1));
            if (size0 > size1) {
                passedFirstCheck.remove(1);
            } else {
                passedFirstCheck.remove(0);
            }
        }


        int leftX = IMAGE_WIDTH;
        int rightX = 0;
        int topY = IMAGE_HEIGHT;
        int bottomY = 0;

        for(MatOfPoint p: passedFirstCheck) {
            //System.out.println("Area: " + Imgproc.contourArea(p));
            resultContours.add(new Contour(p));
            Rect gRect = Imgproc.boundingRect(p);
            Log.v("Y VALUE", Integer.toString(gRect.y+gRect.width));
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

        if(bottomY == 0 && topY == IMAGE_HEIGHT) {
            center.setY(-1);
            center.setX(-1);
            detectedVortex = false;
            detectedTarget = false;
        } else {
            detectedVortex = true;
            center.setY(1.0*(bottomY+topY)/2);
            center.setX((rightX + leftX)/2);
            detectedTarget = true;
        }
        Point vortexCenter = new Point(center.getX(), center.getY());
        Drawing.drawCircle(rgba, vortexCenter, 10, new ColorRGBA(255, 255, 255));
        //Drawing.drawContours(rgba, resultContours, new ColorRGBA(255, 0, 0));
        Drawing.drawRectangle(rgba, new Point(leftX, topY), new Point(rightX, bottomY), new ColorRGBA(255, 255, 0));
        return rgba;
    }

}

