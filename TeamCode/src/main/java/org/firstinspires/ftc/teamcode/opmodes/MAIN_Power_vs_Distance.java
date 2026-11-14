package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Config
@TeleOp(name="MAIN_Power_vs_Distance", group="MAIN")
public class MAIN_Power_vs_Distance extends LinearOpMode {

    // 6000 RPM / 60 SEC/MIN * 28 TICKS/REV
    //private static final double MAX_TICKS = 6000.0 / 60.0 * 28.0;
    //private static final double SERVO1_OPEN = 0.3;
    //private static final double SERVO1_CLOSED = 0.0;
    private static double outtakePower = 0.40;
    public static double kTurn = 0.025;
    // Intake toggle flags
    private boolean intakeToggle = false;
    private boolean lastRB = false;   // previous right bumper state

    boolean manualMode = false;
    boolean xWasPressed = false;

    @Override
    public void runOpMode() {
        // Hardware Mapping
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "fl"); // port 0
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "bl"); // port 1
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "fr"); // port 2
        DcMotor backRight = hardwareMap.get(DcMotor.class, "br"); // port 3

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        Pose2d beginPose = new Pose2d(60, -20, Math.toRadians(90)); // 90
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, false);

        // TODO add flowerScoop motors/servos

        // Motor Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        // TODO add flowerScoop motors

        // Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO add turret, flowerScoop motors

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
            if(gamepad1.dpad_down) {
                outtakePower = 0.64;
            } else if(gamepad1.dpad_up) {
                outtakePower = 0.66;
            } else if(gamepad1.dpad_right) {
                outtakePower = 0.68;
            } else if(gamepad1.dpad_left) {
                outtakePower = 0.70;
            }

            if(gamepad1.left_bumper) {
                outtake.setRPM(6000.0 * outtakePower);
            }
            else if (gamepad1.left_trigger > 0.1) {
                outtake.setRPM(6000.0);
            }
            else outtake.stop();

            // INTAKE CONTROLS
            boolean rb = gamepad1.right_bumper;

            // Toggle intake
            if (rb && !lastRB) {
                intakeToggle = !intakeToggle;
            }
            lastRB = rb;

            // Intake direction
            if (gamepad1.right_trigger > 0.1) {
                intake.reverse();
            }
            else if (intakeToggle) {
                intake.intake();
            }
            else {
                intake.stop();
            }

            // SERVO CONTROL
            // Open servo while reversing intake OR holding A
            if (gamepad1.right_trigger > 0.1 || gamepad1.a) {
                outtake.openFeedServo();
            }
            else {
                outtake.closeFeedServo();
            }

            // TURRET SUBSYSTEM
            drive.updatePoseEstimate();
            Pose2d robotPose = drive.localizer.getPose();

            // Toggle between automatic and manual aiming.
            if (gamepad2.x && !xWasPressed) {
                manualMode = !manualMode;
            }
            xWasPressed = gamepad2.x;

            if (manualMode) {
                // Driver controls turret with right stick.
                turret.manualControl(gamepad2.right_stick_x);
            } else {
                // Normal automatic field aiming.
                turret.update(robotPose);
            }

            telemetry.addData("Turret Mode", manualMode ? "MANUAL" : "AUTO");

            telemetry.addLine("--- TURRET ---");
            telemetry.addData("Robot X", robotPose.position.x);
            telemetry.addData("Robot Y", robotPose.position.y);
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading.toDouble()));

            telemetry.addData("Turret Encoder", turret.getCurrentPosition());
            telemetry.addData("Turret Target", turret.getTargetPosition());

            // MANUAL AUTO AIM
            if(gamepad1.x) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                    LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);

                    // TODO only target certain apriltags or just the red or blue tags
                    // Auto Aim (Limelight Steering)
                    double tagX = tag.getTargetXDegrees();
                    rx = Range.clip(tagX * kTurn, -0.5, 0.5);

                    // Power vs Distance
                    Pose3D pose = tag.getTargetPoseCameraSpace();
                    double distanceMeters = Math.hypot(pose.getPosition().x, pose.getPosition().z);
                    double distanceInches = distanceMeters * 39.37;

                    // TODO add power for distance if needed
                    //if (distanceInches < 24) outtakePower = 0.4;
                    //else if (distanceInches < 36) outtakePower = 0.4;
                    //else if (distanceInches < 48) outtakePower = 0.4;
                    //else outtakePower = 0.4;

                    // Telemetry
                    telemetry.addData("AprilTag ID", tag.getFiducialId());
                    telemetry.addData("Tag X", "%.1f degrees", tagX);
                    telemetry.addData("Auto Turn", "%.2f", rx);
                    telemetry.addData("Distance", "%.1f inches", distanceInches);
                    telemetry.addData("Target Velocity", "%.0f ticks/sec", outtake.getVelocity());
                } else {
                    telemetry.addData("AprilTag", "Not Detected");
                }
            }

            // Driving Calculations
            double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1.0);
            frontLeft.setPower((ly + lx + rx) / denominator);
            backLeft.setPower((ly - lx + rx) / denominator);
            frontRight.setPower((ly - lx - rx) / denominator);
            backRight.setPower((ly + lx - rx) / denominator);

            telemetry.addLine("--- CONTROLS ---");
            telemetry.addData("Right Bumper", "intake full speed");
            telemetry.addData("Right Trigger", "intake REVERSE full speed");
            telemetry.addData("Hold X", "align with AprilTag/set Power vs Distance");
            telemetry.addData("Left Bumper", "outtake variable (starts 0.40) power");
            telemetry.addData("D-Pad", "Select Power Level [D 0.38, L 0.40, U 0.42, R 0.44]");
            telemetry.addData("Hold A", "actuate outtake control servo");
            telemetry.addLine();
            telemetry.addLine("--- TELEMETRY ---");
            telemetry.addData("Intake Power (1.0 max)", intake.getPower());
            telemetry.addData("Outtake Power (1.0 max)", outtakePower);
            telemetry.addData("Outtake Velocity (2800 max)", outtake.getVelocity());
            telemetry.update();

            // TODO add dashboard telemetry

        }
        limelight.stop();
        turret.stop();
        outtake.stop();
    }
}