package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.VOISweeper;

/**
 * Created by Howard on 10/15/16.
 * Intake Task
 */
public class IntakeTask extends TaskThread {

    public volatile double power = 0;
    public volatile int sweepTime = 0;
    private VOISweeper sweeper;
    public volatile boolean oscillate = false;
    CRServo sweeper1, sweeper2, sweeper3;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public IntakeTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void initialize() {
        sweeper1 = opMode.hardwareMap.crservo.get("sweeper1");
        sweeper2 = opMode.hardwareMap.crservo.get("sweeper2");
        sweeper3 = opMode.hardwareMap.crservo.get("sweeper3");
        this.sweeper = new VOISweeper(sweeper1, sweeper2, sweeper3);
        sweeper.setPower(0);
    }

    @Override
    public void run() {
        boolean printed = false;
        boolean print2 = false;
        while(opMode.opModeIsActive() && running) {

            // TeleOp commands
            if (teleOp) {
                if (opMode.gamepad2.dpad_up) {
                    if (!printed) {
                        System.out.println("Up");
                        printed = true;
                        print2 = false;
                    }
                    sweeper.setPower(1);
                } else if (opMode.gamepad2.dpad_down) {
                    if (!printed) {
                        System.out.println("Down");
                        printed = true;
                        print2 = false;
                    }
                    sweeper.setPower(-1);
                } else if (opMode.gamepad2.dpad_left) {
                    System.out.println("Oscillate");
                    printed = false;
                    print2 = false;
                    oscillate = true;
                } else if (opMode.gamepad1.right_trigger > 0) {
                    sweeper.setPower(1);
                } else if (opMode.gamepad1.left_trigger > 0) {
                    sweeper.setPower(-1);
                } else {
                    sweeper.setPower(0);
                    if (!print2) {
                        System.out.println("None");
                        print2 = true;
                        printed = false;
                    }

                }
            }
            // Autonomous commands
            if (power != 0) {
                sweeper.setPower(power);
                power = 0;
                int temp = sweepTime;
                sweepTime = 0;
                sleep(temp);
                sweeper.setPower(0);
            }
            if (oscillate) {
                boolean stoppedPressing = false;
                double pow = 1;
                sweeper.setPower(pow);
                timer.reset();
                while (oscillate && opMode.opModeIsActive()) {
//                    if (teleOp) {
//                        if (!opMode.gamepad2.dpad_left) {
//                            System.out.println(1);
//                            stoppedPressing = true;
//                        }
//                        if (opMode.gamepad2.dpad_left && stoppedPressing) {
//                            System.out.println(2);
//                            while (opMode.gamepad2.dpad_left);
//                            oscillate = false;
//                            break;
//                        }
//                        if (opMode.gamepad2.dpad_up || opMode.gamepad2.dpad_down) {
//                            System.out.println(3);
//                            oscillate = false;
//                        }
//                    }
                    if (timer.time() > 25) {
                        pow = -pow;
                        sweeper.setPower(pow);
                        timer.reset();
                    }
                }
            }


        }
        sweeper.setPower(0);
    }

    public void setPower(double power) {
        sweeper.setPower(power);
    }

}
