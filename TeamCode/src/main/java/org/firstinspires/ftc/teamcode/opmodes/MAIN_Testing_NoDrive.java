package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
@Config
@TeleOp(name = "MAIN_Testing_NoDrive", group="MAIN")
public class MAIN_Testing_NoDrive extends LinearOpMode {

    // 6000 RPM / 60 SEC/MIN * 28 TICKS/REV
    private static final double MAX_TICKS = 6000.0 / 60.0 * 28.0;
    private static double outtakePower = 0.40;
    // Intake toggle flags
    private boolean intakeToggle = false;
    private boolean lastRB = false;   // previous right bumper state

    @Override
    public void runOpMode() {
        // Hardware Mapping
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "outtake");

        // Motor Directions
        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.FORWARD);

        // Zero Power Behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set PIDF coefficients on outtake motors
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(18, 0, 0, 13.5));

        waitForStart();

        while (opModeIsActive()) {

            // OUTTAKE CONTROLS
            if(gamepad1.dpad_down) {
                outtakePower = 0.38;
            } else if(gamepad1.dpad_up) {
                outtakePower = 0.42;
            } else if(gamepad1.dpad_right) {
                outtakePower = 0.44;
            } else if(gamepad1.dpad_left) {
                outtakePower = 0.40;
            }

            if(gamepad1.left_bumper) {
                outtake.setVelocity(MAX_TICKS * outtakePower);
            }
            else if (gamepad1.left_trigger > 0.1) {
                outtake.setVelocity(MAX_TICKS);
            }
            else outtake.setVelocity(0);

            // INTAKE TOGGLE/CONTROLS
            boolean rb = gamepad1.right_bumper;

            // Rising edge detection: only toggles when RB goes from unpressed → pressed
            if (rb && !lastRB) {
                intakeToggle = !intakeToggle;
            }
            lastRB = rb;

            // Apply the toggle state
            if (intakeToggle) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

            if (gamepad1.right_trigger > 0.1){
                intake.setPower(-1.0);
            }

            telemetry.addLine("--- CONTROLS ---");
            telemetry.addData("Right Bumper", "intake full speed");
            telemetry.addData("Right Trigger", "intake REVERSE full speed");
            telemetry.addData("Hold X", "align with AprilTag/set Power vs Distance");
            telemetry.addData("Left Bumper", "outtake variable (starts 0.40) power");
            telemetry.addData("D-Pad", "Select Power Level [D 0.38, L 0.40, U 0.42, R 0.44]");
            telemetry.addLine();
            telemetry.addLine("--- TELEMETRY ---");
            telemetry.addData("Outtake Power (1.0 max)", outtakePower);
            telemetry.addData("Outtake Velocity (2800 max)", outtake.getVelocity());
            telemetry.addData("Intake Power (1.0 max)", intake.getPower());
            telemetry.update();
        }
    }
}