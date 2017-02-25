package org.firstinspires.ftc.teamcode.Tests;


import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VortexCenterPoint;
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
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Tests.VisionShootingTest.VisionMode.PARTICLES;
import static org.firstinspires.ftc.teamcode.Tests.VisionShootingTest.VisionMode.VORTEX;
import static org.firstinspires.ftc.teamcode.robotutil.Team.BLUE;
import static org.firstinspires.ftc.teamcode.robotutil.Team.RED;


/**
 * Created by Stephen on 12/23/2016.
 * Vision Shooting Test
 */

/*
    OpenCV Testing: Uses the LinearOpModeVision class that is supposed to gather image data, supposed to display the images from OpenCV to a camera view on the screen. Untested!!
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "VisionShootingTest", group = "Tests")

public class VisionShootingTest extends LinearOpModeVision {

    Mat mHsvMat;
    Mat red1;
    Mat red2;
    Mat colorMask;
    Mat blurred;
    MatOfInt convexHull;
    MatOfPoint2f temp2fMat;
    MatOfPoint2f polyApprox;
    MatOfInt4 convexityDefects;
    List<MatOfPoint> initialContourList;
    List<MatOfPoint> potentialContours;
    List<Contour> resultContours;
    List<MatOfPoint> passedFirstCheck;
    VortexCenterPoint center;

    boolean detectedVortex = false;


    static final int VORTEX_THRESHOLD = 3500;
    static final int IMAGE_HEIGHT = 600;       //ZTE Camera picture size
    static final int IMAGE_WIDTH = 800;
    static final int NEARNESS_Y = 50;
    static final int NEARNESS_X = 50;
    static final int YCENTER = 300;
    static final double MAX_VORTEX_AREA_RATIO = 0.6;

    static final int ACCEPTABLE_ERROR = 50;
    static final double MIN_PARTICLE_AREA_RATIO = 0.3;
    static final double PARTICLE_MIN_THRESHOLD = 0;
    static final double PARTICLE_MAX_THRESHOLD = 4000;
    static final double correctPower = 0.05;

    MecanumDriveTrain driveTrain;

    Team team = BLUE;
    VisionMode visMode = PARTICLES;


    public enum VisionMode {
        VORTEX, PARTICLES
    }
    @Override
    public void runOpMode() {
        center = new VortexCenterPoint(-1, -1);
        initCamera();   //Start OpenCV
        initVision();   //Do a bunch of initialization for vision code
        //initRobot();
        waitForStart();
        ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()) {
            telemetry.addData("Time: ", t.time());
            telemetry.addData("Y Location", center.getY());
            telemetry.addData("X Location", center.getX());
            //System.out.println("Y Location: " + center.getY());
            telemetry.update();
            // too much to right is too big y, is negative power
            System.out.println("X: " + center.getX() + " Y: " + center.getY());
            if (center.getY() == -1) {
                //driveTrain.startRotation(0.1);
            } else if (center.getY() < IMAGE_HEIGHT/2 - ACCEPTABLE_ERROR) {
                //driveTrain.startRotation(correctPower);
            } else if (center.getY() > IMAGE_HEIGHT/2 + ACCEPTABLE_ERROR) {
                //driveTrain.startRotation(-correctPower);
            } else {
                //driveTrain.stopAll();
            }
        }
        stopCamera();   //Tear down the camera instance
        System.out.println("Camera Stopped");
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {       //Callback from OpenCV leads here
        //Do image processing for individual frames here
        //Convert image to HSV format
        Imgproc.cvtColor(rgba, mHsvMat, Imgproc.COLOR_BGR2HSV_FULL);

        //Define two color ranges to match as red because the hue for red crosses over 180 to 0
        if (team == RED) {
            Core.inRange(mHsvMat, new Scalar(0, 150, 40), new Scalar(10, 255, 255), red1);
            Core.inRange(mHsvMat, new Scalar(160, 150, 40), new Scalar(240, 255, 255), red2);
            Core.bitwise_or(red1, red2, colorMask);

        } else if (team == BLUE) {
            Core.inRange(mHsvMat, new Scalar(90, 25, 40), new Scalar(130, 255, 255), colorMask);
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

            if (Math.abs(x - center.getX()) > 50 || Math.abs(y - center.getY()) > 50) {
                if (detectedVortex) {
                    //continue;
                }
            }
            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 255, 255));

            double contourArea = Imgproc.contourArea(p);

            if (visMode == VORTEX) {

                // center vortex is not very solid shape, so if actual area is greater than 40% of rect area then it can't be vortex
                if (contourArea < VORTEX_THRESHOLD) {
                    continue;
                }
                Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 255, 0));

                if (polyApproxFloat.toArray().length > 4 && rect.x < IMAGE_WIDTH / 2) {
                    if (contourArea/rect.area() < MAX_VORTEX_AREA_RATIO) {
                        Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 0, 255));
                        System.out.println("Contour Area Shooting: " + contourArea);
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
                Drawing.drawContours(rgba, resultContours, new ColorRGBA(255, 0, 0));

                // if correct size
                if (contourArea > PARTICLE_MIN_THRESHOLD && contourArea < PARTICLE_MAX_THRESHOLD) {
                    System.out.println(contourArea);
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
        } else {
            detectedVortex = true;
            center.setY(1.0*(bottomY+topY)/2);
            center.setX((rightX + leftX)/2);
        }
        Point vortexCenter = new Point(center.getX(), center.getY());
        Drawing.drawCircle(rgba, vortexCenter, 10, new ColorRGBA(255, 255, 255));
        Drawing.drawContours(rgba, resultContours, new ColorRGBA(255, 0, 0));
        Drawing.drawRectangle(rgba, new Point(leftX, topY), new Point(rightX, bottomY), new ColorRGBA(255, 255, 255));


        return rgba;
    }

    public void initRobot() {
        //Initialize robot hardware and stuff here
        driveTrain = new MecanumDriveTrain(this);
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

    }

}

