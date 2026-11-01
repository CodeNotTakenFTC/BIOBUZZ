package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
@TeleOp(name = "MAIN_Power_vs_Distance", group="MAIN")
public class MAIN_Power_vs_Distance extends LinearOpMode {

    // 6000 RPM / 60 SEC/MIN * 28 TICKS/REV
    private static final double MAX_TICKS = 6000.0 / 60.0 * 28.0;
    private static double outtakePower = 0.40;

    @Override
    public void runOpMode() {
        // Hardware Mapping
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        // TODO add transfer, turret motors/servos

        // Motor Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        // TODO add transfer, turret motors

        // Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO add transfer, turret motors

        // Set outtake motors to use encoder-based velocity control
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // TODO set PIDF coefficients

        // Limelight Initialization
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // FTC Dashboard and Camera Stream
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(limelight, 30);

        waitForStart();

        while (opModeIsActive()) {

            double ly = -gamepad1.left_stick_y; // Forward/Backward
            double lx = gamepad1.left_stick_x;  // Strafe
            double rx = gamepad1.right_stick_x; // Turn

            // OUTTAKE CONTROLS
            if(gamepad1.dpad_left) {
                outtakePower = 0.40;
            } else if(gamepad1.dpad_up) {
                outtakePower = 0.42;
            } else if(gamepad1.dpad_right) {
                outtakePower = 0.44;
            } else if(gamepad1.dpad_down) {
                outtakePower = 0.38;
            }

            if(gamepad1.right_bumper) {
                outtake.setVelocity(MAX_TICKS * outtakePower);
            }
            else if (gamepad1.right_trigger > 0.1) {
                outtake.setVelocity(MAX_TICKS * 0.50);
            }
            else outtake.setVelocity(0);

            // INTAKE CONTROLS
            if (gamepad1.a) {
                intake.setPower(1.0);
            }
            else if (gamepad1.b){
                intake.setPower(0.5);
            }
            else if (gamepad1.y){
                intake.setPower(-0.5);
            }
            else intake.setPower(0.0);

            // Hold 'x' to auto aim and adjust power for distance
            if(gamepad1.x) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                    LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);

                    // Auto Aim (Limelight Steering)
                    double tagX = tag.getTargetXDegrees();
                    rx = Range.clip(tagX * 0.025, -0.5, 0.5);

                    // Power vs Distance
                    Pose3D pose = tag.getTargetPoseCameraSpace();
                    double distanceMeters = Math.hypot(pose.getPosition().x, pose.getPosition().z);
                    double distanceInches = distanceMeters * 39.37;

                    //double outtakePower;
                    // TODO adjust correctly power for distance
                    if (distanceInches < 24) outtakePower = 0.6;
                    else if (distanceInches < 36) outtakePower = 0.7;
                    else if (distanceInches < 48) outtakePower = 0.8;
                    else outtakePower = 0.9;

                    outtake.setVelocity(MAX_TICKS * outtakePower);

                    // Telemetry
                    telemetry.addData("AprilTag ID", tag.getFiducialId());
                    telemetry.addData("Tag X", "%.1f degrees", tagX);
                    telemetry.addData("Auto Turn", "%.2f", rx);
                    telemetry.addData("Distance", "%.1f inches", distanceInches);
                    telemetry.addData("Target Velocity", "%.0f ticks/sec", outtake.getVelocity());
                } else {
                    outtake.setVelocity(0);
                    telemetry.addData("AprilTag", "Not Detected");
                }
            } else outtake.setVelocity(0);

            // Driving Calculations
            double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1.0);
            frontLeft.setPower((ly + lx + rx) / denominator);
            backLeft.setPower((ly - lx + rx) / denominator);
            frontRight.setPower((ly - lx - rx) / denominator);
            backRight.setPower((ly + lx - rx) / denominator);

            telemetry.addLine("--- CONTROLS ---");
            telemetry.addData("Right Trigger", "outtake 0.50 power");
            telemetry.addData("Right Bumper", "outtake variable (starts 0.40) power");
            telemetry.addData("D-Pad", "Select Power Level");
            telemetry.addData("A", "intake full speed");
            telemetry.addData("B", "intake 1/2 speed");
            telemetry.addData("Y", "intake REVERSE 1/2 speed");
            telemetry.addData("X", "Auto Aim");
            telemetry.addLine();
            telemetry.addLine("--- TELEMETRY ---");
            telemetry.addData("Outtake Power (1.0 max)", outtakePower);
            telemetry.addData("Outtake Velocity (2800 max)", outtake.getVelocity());
            telemetry.addData("Intake Power (1.0 max)", intake.getPower());
            telemetry.update();
        }
        limelight.stop();
    }

}