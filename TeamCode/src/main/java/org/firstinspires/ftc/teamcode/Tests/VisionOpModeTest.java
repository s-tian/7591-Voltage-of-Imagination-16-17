package org.firstinspires.ftc.teamcode.Tests;


import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VortexCenterPoint;
import org.firstinspires.ftc.teamcode.vision.LinearOpModeVision;
import org.lasarobotics.vision.detection.objects.Contour;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.lasarobotics.vision.util.color.Color;


import java.util.ArrayList;
import java.util.List;

/**
 * Created by Stephen on 12/23/2016.
 */

/*
    OpenCV Testing: Uses the LinearOpModeVision class that is supposed to gather image data, supposed to display the images from OpenCV to a camera view on the screen. Untested!!
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "VisionOpModeTest", group = "Tests")




public class VisionOpModeTest extends LinearOpModeVision {

    Mat mHsvMat;
    Mat red1;
    Mat red2;
    Mat redMask;
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

    MecanumDriveTrain driveTrain;

    static final int THRESHOLD = 600;
    static final int IMAGE_HEIGHT = 300;       //ZTE Camera picture size
    static final int IMAGE_WIDTH = 100;
    static final int NEARNESS_Y = 50;
    static final int NEARNESS_X = 50;

    @Override
    public void runOpMode() {
        center = new VortexCenterPoint(-1, -1);
        initCamera();   //Start OpenCV
        initVision();   //Do a bunch of initialization for vision code
        initRobot();
        waitForStart();
        ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()) {
            telemetry.addData("Time: ", t.time());
            telemetry.addData("Y Location", center.getY());
            System.out.println("Y Location: " + center.getY());
            telemetry.update();
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
        Core.inRange(mHsvMat, new Scalar(0, 100, 100), new Scalar(10, 255, 255), red1);
        Core.inRange(mHsvMat, new Scalar(160, 100, 100), new Scalar(179, 255, 255), red2);
        //OR the two masks together to produce a mask that combines the ranges
        //Core.addWeighted(red1, 1.0, red2, 1.0, 0.0, redMask);
        Core.bitwise_or(red1, red2, redMask);
        initialContourList.clear();
        potentialContours.clear();
        passedFirstCheck.clear();
        resultContours.clear();
        //Find the external contours for the red mask using the fast simple approximation
        Imgproc.findContours(redMask, initialContourList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for(MatOfPoint p: initialContourList) {        //Go through preliminary list of contours
            p.convertTo(temp2fMat, CvType.CV_32F);      //Convert MatOfPoint to MatofPoint2f to run approxPolyDP
            double perimeter = Imgproc.arcLength(temp2fMat, true);
            //Approximate each contour using a polygon
            Imgproc.approxPolyDP(temp2fMat, polyApprox, 0.02*perimeter, true);
            MatOfPoint polyApproxFloat = new MatOfPoint(polyApprox.toArray());
            Rect rect = Imgproc.boundingRect(polyApproxFloat);
            int y = rect.y+rect.height;
            int x = rect.x;

            //if(Imgproc.contourArea(p) > THRESHOLD && !Imgproc.isContourConvex(p) && polyApproxFloat.toArray().length > 7 && x < IMAGE_HEIGHT/2) {
//            if(Imgproc.contourArea(p) > THRESHOLD && Imgproc.contourArea(p) < 0.5*rect.area() && polyApproxFloat.toArray().length > 4 && !Imgproc.isContourConvex(p) && x < IMAGE_HEIGHT/2) {
//                passedFirstCheck.add(p);
//            } else if (x < IMAGE_HEIGHT/2 && polyApproxFloat.toArray().length > 4) {
//                potentialContours.add(p);
//            }
            //System.out.println(polyApproxFloat.toArray().length);
            if(polyApproxFloat.toArray().length > 6 && x < IMAGE_HEIGHT/2) {
                if(Imgproc.contourArea(p) > THRESHOLD) {
                    Imgproc.convexHull(polyApproxFloat, convexHull);
                    if(convexHull.rows() > 2) {
                        Imgproc.convexityDefects(polyApproxFloat, convexHull, convexityDefects);
                        List<Integer> cdlist = convexityDefects.toList();
                        System.out.println("Points: " + convexHull.rows());
                        int count = 0;
                        for(int i = 0; i < cdlist.size(); i+=4) {
                            double depth = cdlist.get(i+3)/256.0;
                            if (depth > Math.sqrt(rect.height)/2) {
                                count++;
                            }
                        }
                        if (count > 0) {
                            passedFirstCheck.add(p);
                        }
                        System.out.println("Count: " + count + " Size: " + cdlist.size());
                    }

                } else {
                    potentialContours.add(p);
                }
            }
        }

        int leftX = IMAGE_WIDTH;
        int rightX = 0;
        int topY = IMAGE_HEIGHT;
        int bottomY = 0;

        for(MatOfPoint p: passedFirstCheck) {
            resultContours.add(new Contour(p));
            Rect gRect = Imgproc.boundingRect(p);
            Log.e("YVALUE", Integer.toString(gRect.y+gRect.width));
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

//            for(MatOfPoint q: potentialContours) {
//                Rect mRect = Imgproc.boundingRect(q);
//                int distY = Math.abs(mRect.y+mRect.height-gRect.y-gRect.height);
//                int distX;
//                if(mRect.x < gRect.x) {
//                    distX = Math.abs(mRect.x+mRect.width-gRect.x);
//                } else {
//                    distX = Math.abs(gRect.x+gRect.width-mRect.x);
//                }
//                if(Math.abs(distY) < NEARNESS_Y && Math.abs(distX) < NEARNESS_X) {
//                    resultContours.add(new Contour(q));
//                    if(mRect.x+mRect.width > rightX) {
//                        rightX = mRect.x+mRect.width;
//                    }
//                    if(mRect.x < leftX) {
//                        leftX = mRect.x;
//                    }
//                    if(mRect.y+mRect.height > bottomY) {
//                        bottomY = mRect.y+mRect.height;
//                    }
//                    if(mRect.y < topY) {
//                        topY = mRect.y;
//                    }
//                }
//            }
        }

        if(bottomY == 0 && topY == IMAGE_HEIGHT) {
            center.setY(-1);
        } else {
            center.setY(1.0*(bottomY+topY));
        }
        Point vortexCenter = new Point((rightX + leftX)/2, (topY+bottomY)/2);
        Drawing.drawContours(rgba, resultContours, new ColorRGBA(255, 0, 0), 2);
        Drawing.drawCircle(rgba, vortexCenter, 10, new ColorRGBA(255, 255, 255));
        return rgba;
    }

    public void initRobot() {
        //Initialize robot hardware and stuff here
    }

    public void initVision() {
        mHsvMat = new Mat();
        red1 = new Mat();
        red2 = new Mat();
        redMask = new Mat();
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
