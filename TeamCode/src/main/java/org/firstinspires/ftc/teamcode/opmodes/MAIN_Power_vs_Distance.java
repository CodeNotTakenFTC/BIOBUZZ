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

    @Override
    public void runOpMode() {
        // Hardware Mapping
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right");
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        // TODO add transfer, turret motors
        // TODO add servos

        // Motor Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        // TODO add transfer, turret motors

        // Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO add intake, transfer, turret motors

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

            // Hold 'a' to auto aim and adjust power for distance
            if(gamepad1.a) {
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

                    double outtakePower;
                    if (distanceInches < 24) outtakePower = 0.6;
                    else if (distanceInches < 36) outtakePower = 0.7;
                    else if (distanceInches < 48) outtakePower = 0.8;
                    else outtakePower = 0.9;

                    double targetVelocity = outtakePower * MAX_TICKS;
                    outtake.setVelocity(targetVelocity);

                    // Telemetry
                    telemetry.addData("AprilTag ID", tag.getFiducialId());
                    telemetry.addData("Tag X", "%.1f degrees", tagX);
                    telemetry.addData("Auto Turn", "%.2f", rx);
                    telemetry.addData("Distance", "%.1f inches", distanceInches);
                    telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocity);
                } else {
                    outtake.setVelocity(0);
                    telemetry.addData("AprilTag", "Not Detected");
                }

            } else if(gamepad1.b) {
                outtake.setVelocity(1400);
            } else if (gamepad1.x) {
                outtake.setVelocity(2800);
            } else outtake.setVelocity(0);

            // TODO outtake, intake, transfer, turret motor controls
            if (gamepad1.right_trigger > 0.1) {
                double intakePower = 1.0;
                intake.setPower(intakePower);
            }
            else {
                intake.setPower(0.0);
            }

            // DRIVING CALCULATIONS
            double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1.0);
            frontLeft.setPower((ly + lx + rx) / denominator);
            backLeft.setPower((ly - lx + rx) / denominator);
            frontRight.setPower((ly - lx - rx) / denominator);
            backRight.setPower((ly + lx - rx) / denominator);

            telemetry.update();
        }
        limelight.stop();
    }
}