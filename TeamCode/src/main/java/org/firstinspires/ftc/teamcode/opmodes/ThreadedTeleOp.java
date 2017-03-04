package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.DriveTrainTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;
import org.firstinspires.ftc.teamcode.tasks.TaskThread;
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

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.opmodes.VisionShootingTest.VisionMode.PARTICLES;
import static org.firstinspires.ftc.teamcode.robotutil.Team.BLUE;
import static org.firstinspires.ftc.teamcode.robotutil.Team.RED;

/**
 * Created by bunnycide on 11/4/16.
 * TeleOp
 */

@TeleOp(name="Threaded Teleop", group = "Drive")

public class ThreadedTeleOp extends LinearOpModeVision {

    private Servo guide;
    private DriveTrainTask driveTrainTask;
    private FlywheelTask flywheelTask;
    private CapBallTask capBallTask;
    private IntakeTask intakeTask;
    private ButtonPusherTask buttonPusherTask;

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

    boolean aiming = false;
    boolean detectedVortex = false;

    int centX = -1;
    int centY = -1;
    double zeroAngle = 0;
    static final int ACCEPTABLE_ERROR = 50;
    static final int PARTICLE_TARGET = 320;

    double correctPower = -0.03;

    static final int TARGET_VORTEX_HEIGHT = 600;
    static final int VORTEX_THRESHOLD = 8000;
    static final int IMAGE_HEIGHT = 600;       //ZTE Camera picture size
    static final int IMAGE_WIDTH = 800;
    static final double MAX_VORTEX_AREA_RATIO = 0.6;

    // this is for vision
    static int minVortexBlueH = 96;
    static int maxVortexBlueH = 120;
    static int minVortexBlueSat = 50;
    static int maxVortexBlueVal = 200;

    static int minBlueH = 80;
    static int maxBlueH = 110;
    static int minRedH = 160;
    static int maxRedH = 200;
    static int minBlueSat = 125;
    static int minRedSat = 50;
    static int minBlueVal = 75;
    static int minRedVal = 50;

    int cameraNumber = 1;
    MecanumDriveTrain driveTrain;
    boolean detectedTarget = false;
    ElapsedTime rejectTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int rejectTime = 500;
    ElapsedTime pauseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    Team team = BLUE;
    VisionShootingTest.VisionMode visMode = PARTICLES;

    boolean ballDetected = false;

    // location stuff
    double robotX = 36;
    double robotY = 24;
    double robotAngle = 0;
    double vortexX = 80;
    double vortexY = 64;

    // general autonomous stuff
    private int shootTime = 2500;

    @Override
    public void runOpMode() {

        initialize();
        initVision();

        telemetry.addData("Ready!", "");
        telemetry.update();
        waitForStart();
        long startTime = System.nanoTime();

        driveTrainTask.start();
        flywheelTask.start();
        intakeTask.start();
        capBallTask.start();
        buttonPusherTask.start();

        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(3);
        while(opModeIsActive()) {
            //Timer for 2 minute teleop period
            long elapsed = System.nanoTime() - startTime;

            if (elapsed > 120 * 1000000000L) {
                //Stop all tasks, the tasks will stop motors etc.
                driveTrainTask.running = false;
                buttonPusherTask.running = false;
                flywheelTask.running = false;
                intakeTask.running = false;
                capBallTask.running = false;
                //Get out of the loop
                break;
            } else {
                int seconds = 120 - (int) (elapsed/1000000000L);
                String timeString = (seconds/60) + ":";
                if (seconds%60 < 10) {
                    timeString += 0;
                }
                timeString += seconds%60;
                telemetry.addData("Time elapsed", timeString);
                telemetry.addData("Left error", df.format(flywheelTask.currentErrorLeft*100));
                telemetry.addData("Right error", df.format(flywheelTask.currentErrorRight*100));
                telemetry.addData("Flywheel state", flywheelTask.state);
            }

            if (gamepad1.x) {
                aiming = true;
            }
            if (gamepad1.y || driveTrainTask.moving) {
                aiming = false;
            }
            driveTrainTask.aiming = this.aiming;
            if (aiming) {
                rotateAim();
            }

            telemetry.update();
        }
    }

    public void initialize(){
        guide = hardwareMap.servo.get("guide");
        guide.setPosition(ButtonPusherTask.upPosition);
        driveTrainTask = new DriveTrainTask(this);
        flywheelTask = new FlywheelTask(this);
        intakeTask = new IntakeTask(this);
        capBallTask = new CapBallTask(this);
        buttonPusherTask = new ButtonPusherTask(this);
        TaskThread.calculateVoltage(this);
        buttonPusherTask.teleOp = intakeTask.teleOp = flywheelTask.teleOp = true;
        intakeTask.flywheelTask = flywheelTask;
        initCamera(cameraNumber);   //Start OpenCV
        driveTrain = new MecanumDriveTrain(this);
    }

    public Mat processFrame(Mat rgba, Mat gray) {
        if (aiming) {
            flywheelTask.setPhoneVision();
            Imgproc.GaussianBlur(rgba, rgba, new Size(5, 5), 2, 2);
            Drawing.drawRectangle(rgba, new Point(1, 1), new Point(IMAGE_WIDTH - 1, IMAGE_HEIGHT - 1), new ColorRGBA(255, 255, 255));

            return findVortex(rgba, gray);
        } else {
            return rgba;
        }
    }

    Mat findVortex(Mat rgba, Mat gray) {
        Imgproc.GaussianBlur(rgba, rgba, new Size(5, 5), 2, 2);
        Imgproc.cvtColor(rgba, mHsvMat, Imgproc.COLOR_RGB2HSV);
        Mat filtered = new Mat();
        if (team == BLUE) {
            Core.inRange(mHsvMat, new Scalar(minVortexBlueH, minVortexBlueSat, 50), new Scalar(maxVortexBlueH, 255, maxVortexBlueVal), filtered);
        } else if (team == RED) {
            Core.inRange(mHsvMat, new Scalar(minRedH, minRedSat, minRedVal), new Scalar(255, 255, 255), filtered);
        }
        Imgproc.GaussianBlur(filtered, filtered, new Size(5, 5), 10);
        initialContourList.clear();
        potentialContours.clear();
        passedFirstCheck.clear();
        resultContours.clear();
        //Find the external contours for the red mask using the fast simple approximation
        Imgproc.findContours(filtered, initialContourList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double minArea = VORTEX_THRESHOLD;
        for(MatOfPoint p: initialContourList) {        //Go through preliminary list of contours
            p.convertTo(temp2fMat, CvType.CV_32F);      //Convert MatOfPoint to MatofPoint2f to run approxPolyDP
            double perimeter = Imgproc.arcLength(temp2fMat, true);
            //Approximate each contour using a polygon
            double contourArea = Imgproc.contourArea(p);
            Imgproc.approxPolyDP(temp2fMat, polyApprox, 0.02*perimeter, true);
            MatOfPoint polyApproxFloat = new MatOfPoint(polyApprox.toArray());
            Rect rect = Imgproc.boundingRect(polyApproxFloat);
            int y = rect.y + rect.height/2;
            int x = rect.x + rect.width/2;
            if (centX != -1) {
                if (Math.abs(x-centX) > 100 || Math.abs(y-centY) > 100) {
                    continue;
                }
            }

            if(polyApproxFloat.toArray().length > 4) {
                if(contourArea > minArea) {
                    Imgproc.convexHull(polyApproxFloat, convexHull);
                    if(convexHull.rows() > 2) {
                        Imgproc.convexityDefects(polyApproxFloat, convexHull, convexityDefects);
                        List<Integer> cdlist = convexityDefects.toList();
                        int count = 0;
                        for(int i = 0; i < cdlist.size(); i+=4) {
                            double depth = cdlist.get(i+3)/256.0;
                            if (depth > (rect.height)/2) {
                                count++;
                            }
                        }
                        if (count > 0) {
                            minArea = contourArea;
                            passedFirstCheck.clear();
                            passedFirstCheck.add(p);
                            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 255, 255));
                        }
                    }

                } else {
                    potentialContours.add(p);
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


        if (passedFirstCheck.isEmpty()) {
            centY = centX = -1;
        } else {
            MatOfPoint p = passedFirstCheck.get(0);
            Rect rect = Imgproc.boundingRect(p);
            resultContours.add(new Contour(p));
            centX = rect.x + rect.width/2;
            centY = rect.y + rect.height/2;
        }

        Drawing.drawContours(rgba, resultContours, new ColorRGBA(255, 0, 0), 2);
        Drawing.drawCircle(rgba, new Point(centX, centY), 10, new ColorRGBA(255, 255, 255));
        return rgba;
    }

    public void rotateAim() {
        flywheelTask.setPhoneVision();

        if (centY < PARTICLE_TARGET - ACCEPTABLE_ERROR) {
            ballDetected = false;
            driveTrain.startRotation(correctPower);
        } else if (centY > PARTICLE_TARGET + ACCEPTABLE_ERROR) {
            ballDetected = false;
            driveTrain.startRotation(-correctPower);
        } else {
            driveTrain.stopAll();
            flywheelTask.setPhoneDown();
            aiming = false;
        }
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

}
