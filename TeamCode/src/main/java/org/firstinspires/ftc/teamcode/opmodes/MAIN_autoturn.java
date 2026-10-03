package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "MAIN_autoturn", group="MAIN")
public class MAIN_autoturn extends LinearOpMode {
    // 1. Declare variables for all 4 motors
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;

    @Override
    public void runOpMode() {
        // 2. Hardware Mapping (Must match your Hub configuration names exactly)
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");

        // 3. Reverse the left side motors because they face the opposite direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // LIMELIGHT INIT
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Mecanum + Limelight Ready");
        telemetry.addLine("Hold A to auto-aim");
        telemetry.update();

        // --- FTC Dashboard and camera stream --- //
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(limelight, 30);

        waitForStart();

        while (opModeIsActive()) {

            // 4. Read Gamepad Input
            // Note: Left stick Y controls forward/back, X controls strafe. Right stick X controls turning.
            double y = -gamepad1.left_stick_y; // Inverted by default
            double x =  gamepad1.left_stick_x;  // Strafe
            double rx = gamepad1.right_stick_x; // Turn

            // LIMELIGHT AUTO-AIM
            boolean tagFound = false;

            if (gamepad1.a) {

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {

                        tagFound = true;

                        double tagX = tag.getTargetXDegrees();

                        // Auto-aim turning
                        rx = tagX * 0.02;

                        // Limit turning speed
                        if (rx > 0.5) rx = 0.5;
                        if (rx < -0.5) rx = -0.5;

                        telemetry.addData("Tag ID", tag.getFiducialId());
                        telemetry.addData("Tag X", "%.1f", tagX);
                        telemetry.addData("Auto Turn", "%.2f", rx);

                        break;
                    }
                }

                // No tag = stop turning
                if (!tagFound) {
                    rx = 0;
                }
            }

            // 5. Calculate power vectors using Mecanum kinematics
            // The denominator ensures the values are normalized and never exceed 1.0
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // 6. Send calculated power to the motors
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Send diagnostic feedback to the Driver Station
            telemetry.addData("Auto Aim", gamepad1.a ? "ON" : "OFF");
            telemetry.addData("Tag Found", tagFound);
            telemetry.update();
        }
        limelight.stop();
    }
}