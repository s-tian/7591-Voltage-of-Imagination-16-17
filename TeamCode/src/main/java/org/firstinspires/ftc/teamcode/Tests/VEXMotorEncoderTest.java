package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.robotutil.VEX393Encoder;

/**
 * Created by Stephen on 1/22/2017.
 */
@TeleOp(name = "VEX Encoder Test", group = "Test")

public class VEXMotorEncoderTest extends LinearOpMode {
    CRServo vexMotor;

    @Override
    public void runOpMode() {
        vexMotor = hardwareMap.crservo.get("sweeper1");
        VEX393Encoder encoder = new VEX393Encoder(hardwareMap, "vexencoder");
        waitForStart();
        while(opModeIsActive()) {
            vexMotor.setPower(-gamepad1.left_stick_y);
            telemetry.addData("Encoder Ticks ", encoder.readRotation());
            telemetry.addData("Motor speed ", encoder.getSignedVelocity());
        }

    }
}
