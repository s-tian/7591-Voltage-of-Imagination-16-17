package org.firstinspires.ftc.teamcode.opmodes;


import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VortexCenterPoint;
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
    Mat red1;
    Mat red2;
    Mat colorMask;
    Mat grayMask;
    Mat lowBrightMask;
    Mat lowSatMask;
    Mat blurred;
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

    boolean detectedVortex = false;


    static final int VORTEX_THRESHOLD = 3500;
    static final int IMAGE_HEIGHT = 600;       //ZTE Camera picture size
    static final int IMAGE_WIDTH = 800;
    static final double MAX_VORTEX_AREA_RATIO = 0.6;

    static final int ACCEPTABLE_ERROR = 50;
    static final double MIN_PARTICLE_AREA_RATIO = 0.6;
    static final double PARTICLE_MIN_THRESHOLD = 1000;
    static final double PARTICLE_MAX_THRESHOLD = 15000;
    double correctPower = 0.05;

    double[] blackArray = new double[] {0, 0, 0, 0};
    Scalar blackScalar = new Scalar(0, 0, 0);


    static int minBlueH = 80;
    static int maxBlueH = 110;
    static int minRedH = 160;
    static int maxRedH = 180;
    static int minBlueSat = 150;
    static int minRedSat = 50;
    static int minBlueBright = 150;
    static int minRedBright = 50;

    int cameraNumber = 1;
    MecanumDriveTrain driveTrain;
    boolean detectedTarget = false;

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
        //initRobot();
        //intakeTask.start();
        waitForStart();

        ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        if (cameraNumber == 1) {
            correctPower *= -1;
        }
        while(opModeIsActive()) {
            telemetry.addData("Time: ", t.time());
            telemetry.addData("Y Location", center.getY());
            telemetry.addData("X Location", center.getX());
            //System.out.println("Y Location: " + center.getY());
            telemetry.update();
            // too much to right is too big y, is negative power
            //System.out.println("X: " + center.getX() + " Y: " + center.getY());
            //rotateAim();
        }
        stopCamera();   //Tear down the camera instance
        System.out.println("Camera Stopped");
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {       //Callback from OpenCV leads here
        //Do image processing for individual frames here
        //Convert image to HSV format

        Imgproc.GaussianBlur(rgba, rgba, new Size(15, 15), 8, 8);

        Imgproc.cvtColor(rgba, mHsvMat, Imgproc.COLOR_RGB2HSV);

        //Define two color ranges to match as red because the hue for red crosses over 180 to 0
        if (visMode == PARTICLES) {
            tileFilter(rgba);
        }


        if (team == RED) {
            //Core.inRange(mHsvMat, new Scalar(0, 50, 40), new Scalar(10, 255, 255), red1);
            Core.inRange(mHsvMat, new Scalar(minRedH, minRedSat, minRedBright), new Scalar(maxRedH, 255, 255), colorMask);
            //Core.bitwise_or(red1, red2, colorMask);

        } else if (team == BLUE) {
            if (visMode == VORTEX) {
                Core.inRange(mHsvMat, new Scalar(minBlueH, minBlueSat, minBlueBright), new Scalar(maxBlueH, 255, 255), colorMask);
            } else if (visMode == PARTICLES) {
                Core.inRange(mHsvMat, new Scalar(minBlueH, minBlueSat, minBlueBright), new Scalar(maxBlueH, 255, 255), colorMask);
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
            // TODO: 2/24/17 experiment with perimeter and convexity defects for restrictions
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
        Drawing.drawContours(rgba, resultContours, new ColorRGBA(255, 0, 0));
        Drawing.drawRectangle(rgba, new Point(leftX, topY), new Point(rightX, bottomY), new ColorRGBA(255, 255, 0));


        return rgba;

    }

    public void initRobot() {
        //Initialize robot hardware and stuff here
        driveTrain = new MecanumDriveTrain(this);
        intakeTask = new IntakeTask(this);
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
        lowBrightMask = new Mat();
        lowSatMask = new Mat();

    }
    public void rotateAim() {

        if (center.getY() == -1) {
            driveTrain.startRotation(0.1);
        } else if (center.getY() < IMAGE_HEIGHT/2 - ACCEPTABLE_ERROR) {
            driveTrain.startRotation(correctPower);
        } else if (center.getY() > IMAGE_HEIGHT/2 + ACCEPTABLE_ERROR) {
            driveTrain.startRotation(-correctPower);
        } else {
            if (detectedTarget) {
                driveForward();
            }
        }
    }

    public void driveForward() {
        if (center.getX() > IMAGE_WIDTH * 0.75) {
            driveTrain.moveForwardNInch(0.2, 10, 5, false, true, false);
        } else {
            intakeTask.setPower(1);
            driveTrain.powerAllMotors(0.25);
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
        Core.inRange(mHsvMat, new Scalar(0, 0, 175), new Scalar(255, 255, 255), lowBrightMask);
        Core.bitwise_or(lowSatMask, lowBrightMask, grayMask);
        Core.bitwise_or(lowMask, grayMask, grayMask);
        Core.bitwise_or(highMask, grayMask, grayMask);
        Imgproc.findContours(grayMask, grayContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < grayContours.size(); i ++) {
            MatOfPoint p = grayContours.get(i);
            Rect rect = Imgproc.boundingRect(p);
            int n = rect.x + rect.width/2;
            if (Imgproc.contourArea(p) > Math.max(PARTICLE_MAX_THRESHOLD * n/IMAGE_WIDTH, 2000)) {
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
                diff = 5;
            }
            if (gamepad1.dpad_down && !downPressed) {
                downPressed = true;
                diff = -5;
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
                    telemetry.addData("Modifying", "Min Blue Bright");
                    minBlueBright += diff;
                    diff = 0;
                    break;
                case 7:
                    telemetry.addData("Modifying", "Min Red Bright");
                    minRedBright += diff;
                    diff = 0;
                    break;

            }

            telemetry.addData("Min Blue", minBlueH);
            telemetry.addData("Max Blue", maxBlueH);
            telemetry.addData("Min Red", minRedH);
            telemetry.addData("Max Red", maxRedH);
            telemetry.addData("Min Blue Sat", minBlueSat);
            telemetry.addData("Min Red Sat", minRedSat);
            telemetry.addData("Min Blue Bright", minBlueBright);
            telemetry.addData("Min Red Bright", minRedBright);
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

}

