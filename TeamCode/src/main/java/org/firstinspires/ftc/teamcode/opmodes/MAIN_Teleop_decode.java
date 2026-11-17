package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Disabled
@TeleOp(name="MAIN_Teleop_decode", group="Linear OpMode")
public class MAIN_Teleop_decode extends LinearOpMode {

    // velocity based shooting and PIDF coefficients
    private double maxTicks = 6000.0 / 60.0 * 28.0;   //based on 6000rpm motor
    public static double P = 18.0; //tuned with FTC Dashboard for fast spin up
    public static double I = 0.0; //stays zero
    public static double D = 0.0; //stays zero
    public static double F = 13.5; //tuned with FTC Dashboard for quick recovery
    // AprilTag fields
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    // Auto-align flag and tunable gains
    private boolean autoAlign = false;
    public static double kTurn   = 0.060;           //GOOD FOR TILES
    public static double alignThreshold = 2.0;
    public static double turnClip = 0.4;            //GOOD FOR TILES
    // Intake toggle flags
    private boolean intakeToggle = false;
    private boolean lastRB = false;   // previous right bumper state
    // Motor powers for telemetry and dashboard adjustment
    private double motorPwr = 0.80;
    private double outtakePwr = 0.45;
    private double intakePwr = 0.0;
    private double beltPwr = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // drive motors
        DcMotor lft_fnt = hardwareMap.dcMotor.get("lf");
        DcMotor lft_bck = hardwareMap.dcMotor.get("lb");
        DcMotor rgt_fnt = hardwareMap.dcMotor.get("rf");
        DcMotor rgt_bck = hardwareMap.dcMotor.get("rb");

        // gameplay motors
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor belt = hardwareMap.dcMotor.get("belt");
        DcMotorEx outtakeLeft = hardwareMap.get(DcMotorEx.class, "outLeft");
        DcMotorEx outtakeRight = hardwareMap.get(DcMotorEx.class, "outRight");

        //set PIDF coefficients on outtake motors
        outtakeLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        outtakeRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        // motor settings, braking and reversed
        lft_fnt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lft_bck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rgt_fnt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rgt_bck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rgt_bck.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- AprilTag Vision Setup ---
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // --- FTC Dashboard and camera stream --- //
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(visionPortal, 30);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // update PIDF coefficients using FTC Dashboard
            PIDFCoefficients left = outtakeLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            if (left.p != P || left.i != I || left.d != D || left.f != F) {
                PIDFCoefficients coefficients = new PIDFCoefficients(P, I, D, F);
                outtakeLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
                outtakeRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            }

            // joystick movement mecanum
            double ly = gamepad1.left_stick_y;      // forward/back
            double lx = gamepad1.left_stick_x;      // strafe
            double rx = gamepad1.right_stick_x;     // turn

            // -------------------------------------------------------
            //      APRILTAG ALIGNMENT
            // -------------------------------------------------------

            // Enable / Disable alignment
            if (gamepad1.right_stick_button) autoAlign = true;
            if (gamepad1.left_stick_button)  autoAlign = false;

            List<AprilTagDetection> detections = tagProcessor.getDetections();
            AprilTagDetection tag = null;

            boolean tagVisible = false;
            double bearing =  0;  // degrees left/right from center

            if (detections != null && !detections.isEmpty()) {
                AprilTagDetection d = detections.get(0);

                if (d != null && d.ftcPose != null) {
                    tag = d;
                    tagVisible = true;
                    bearing = d.ftcPose.bearing;
                } else {
                    // pose not ready yet → treat as NOT visible
                    tagVisible = false;
                }
            }

            double correctedBearing = bearing;

            if (autoAlign && tagVisible) {

                // --- 1) Compute offset based on tag + range ---
                double bearingOffset = 0;  // default no offset

                // Only apply far-shot compensation if distance > 7.5 ft (90 in)
                if (tag.ftcPose.range > 90) {

                    if (tag.id == 20) {         // blue goal tag
                        // Needs to aim LEFT → positive offset (turns CCW)
                        bearingOffset = 3.0;   // adjust as needed
                    }
                    else if (tag.id == 24) {    // red goal tag
                        // Needs to aim RIGHT → negative offset (turns CW)
                        bearingOffset = -3.0;   // adjust as needed
                    }
                }

                // --- 2) Apply offset to the bearing used for steering ---
                correctedBearing = bearing + bearingOffset;

                // scale turning speed by square root to prevent oscillation
                double scaled = Math.signum(correctedBearing) * Math.sqrt(Math.abs(correctedBearing));
                double turnCmd = Range.clip(scaled * kTurn, -turnClip, turnClip);

                // Override driver turning
                rx = -turnCmd;
                ly = 0;  // no forward motion during align
                lx = 0;  // no strafing

                // Stop when aligned
                if (Math.abs(correctedBearing) < alignThreshold) {
                    autoAlign = false;
                }

                // --- DRIVER OVERRIDE ---
                // if driver moves joysticks significantly, cancel autoAlign
                if (Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 ||
                        Math.abs(gamepad1.right_stick_x) > 0.1) {
                    autoAlign = false;
                }
            }

            // --------------------------------------------------
            //     SHOOTING AND DRIVING CONTROLS
            // --------------------------------------------------

            // --- SHOOTING CONTROLS --- //

            // 1) outtake power selection override with D-pad
            if (gamepad1.dpad_left) outtakePwr = 0.425;
            if (gamepad1.dpad_down) outtakePwr = 0.45;
            if (gamepad1.dpad_right) outtakePwr = 0.475;
            if (gamepad1.dpad_up) outtakePwr = 0.5175;

            // 2) recompute target velocity from the potentially new outtakePwr
            double targetVelocity = outtakePwr * maxTicks;

            // 3) set drive motors based on outtakePwr
            if (gamepad1.left_bumper) {
                outtakeLeft.setVelocity(targetVelocity);
                outtakeRight.setVelocity(targetVelocity);
            } else {
                outtakeLeft.setVelocity(0);
                outtakeRight.setVelocity(0);
            }

            // reverse full power outtake for clearing
            if (gamepad1.y) {
                outtakeLeft.setVelocity(-2800);
                outtakeRight.setVelocity(-2800);
            }

            // ----- Intake Toggle -----
            boolean rb = gamepad1.right_bumper;

            // Rising edge detection: only toggles when RB goes from unpressed → pressed
            if (rb && !lastRB) {
                intakeToggle = !intakeToggle;
            }
            lastRB = rb;

            // Apply the toggle state
            if (intakeToggle) {
                intakePwr = 1.0;
            } else {
                intakePwr = 0.0;
            }
            intake.setPower(intakePwr);

            // belt power
            if (gamepad1.x) {
                beltPwr = 0.6;
                belt.setPower(beltPwr);
            }
            else {
                beltPwr = 0;
                belt.setPower(beltPwr);
            }

            // intake and belt lower power
            if (gamepad1.a) {
                beltPwr = 0.5;
                intakePwr = 0.5;
                belt.setPower(beltPwr);
                intake.setPower(intakePwr);
            }

            // reverse belt, intake, and outtake to clear balls from wheel
            if (gamepad1.b) {
                beltPwr = -0.5;
                intakePwr = -0.5;
                belt.setPower(beltPwr);
                intake.setPower(intakePwr);
                outtakeLeft.setVelocity(-targetVelocity / 2);
                outtakeRight.setVelocity(-targetVelocity / 2);
            }

            // --- DRIVING CONTROLS --- //

            double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1.0);

            double frontLeftPower  = (ly - lx - rx) / denominator;
            double backLeftPower   = (ly + lx - rx) / denominator;
            double frontRightPower = (ly + lx + rx) / denominator;
            double backRightPower  = (ly - lx + rx) / denominator;

            lft_fnt.setPower(frontLeftPower * motorPwr);
            lft_bck.setPower(backLeftPower * motorPwr);
            rgt_fnt.setPower(frontRightPower * motorPwr);
            rgt_bck.setPower(backRightPower * motorPwr);

            // standard telemetry
            telemetry.addData("Drive Power   ", motorPwr);
            telemetry.addData("Intake Power  ", intakePwr);
            telemetry.addData("Outtake Power ", outtakePwr);
            telemetry.addData("Belt Power    ", beltPwr);
            telemetry.addData("Outtake Velocity ", outtakeLeft.getVelocity());
            telemetry.addData("Target Velocity ", targetVelocity);

            // AprilTag alignment telemetry
            telemetry.addLine("============");
            telemetry.addData("Tag Visible   ", tagVisible);
            telemetry.addData("Bearing (deg) ", bearing);
            telemetry.addData("Corrected Bearing (deg) ", correctedBearing);
            telemetry.addData("AutoAlign     ", autoAlign);
            telemetry.addData("kTurn         ", kTurn);
            telemetry.addData("alignThreshold", alignThreshold);

            if (tagVisible) {
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Range (in)", tag.ftcPose.range);
            }

            // FTC Dashboard telemetry for graphing
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("TargetVelocity", targetVelocity);
            packet.put("Outtake Velocity", outtakeLeft.getVelocity());
            packet.put("Bearing", bearing);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
