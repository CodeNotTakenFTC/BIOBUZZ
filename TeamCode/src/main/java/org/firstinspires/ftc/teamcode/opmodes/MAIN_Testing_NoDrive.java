package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name = "MAIN_Testing_NoDrive", group="MAIN")
public class MAIN_Testing_NoDrive extends LinearOpMode {

    // 6000 RPM / 60 SEC/MIN * 28 TICKS/REV
    private static final double MAX_TICKS = 6000.0 / 60.0 * 28.0;
    private static double outtakePower = 0.40;

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

            telemetry.addLine("--- CONTROLS ---");
            telemetry.addData("Right Trigger", "outtake 0.50 power");
            telemetry.addData("Right Bumper", "outtake variable (starts 0.40) power");
            telemetry.addData("D-Pad", "Select Power Level");
            telemetry.addData("A", "intake full speed");
            telemetry.addData("B", "intake 1/2 speed");
            telemetry.addData("Y", "intake REVERSE 1/2 speed");
            telemetry.addLine();
            telemetry.addLine("--- TELEMETRY ---");
            telemetry.addData("Outtake Power (1.0 max)", outtakePower);
            telemetry.addData("Outtake Velocity (2800 max)", outtake.getVelocity());
            telemetry.addData("Intake Power (1.0 max)", intake.getPower());
            telemetry.update();
        }
    }
}