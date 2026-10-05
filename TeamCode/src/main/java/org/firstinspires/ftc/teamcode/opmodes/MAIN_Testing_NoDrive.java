package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "MAIN_Testing_NoDrive", group="MAIN")
public class MAIN_Testing_NoDrive extends LinearOpMode {

    // 6000 RPM / 60 SEC/MIN * 28 TICKS/REV
    private static final double MAX_TICKS = 6000.0 / 60.0 * 28.0;

    @Override
    public void runOpMode() {
        // Hardware Mapping
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        // TODO add transfer, turret motors
        // TODO add servos

        // Motor Directions
        outtake.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        // TODO add transfer, turret motors

        // Zero Power Behavior
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO add intake, transfer, turret motors

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.b) {
                outtake.setVelocity(1400);
            }
            else if (gamepad1.x) {
                outtake.setVelocity(2800);
            }
            else outtake.setVelocity(0);

            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(1.0);
            }
            else if (gamepad1.right_bumper){
                intake.setPower(0.5);
            }
            else intake.setPower(0.0);

            telemetry.addData("B: ", "outtake 1/2 speed");
            telemetry.addData("X: ", "outtake full speed");
            telemetry.addData("Right Trigger: ", "intake full speed");
            telemetry.addData("Right Bumper: ", "intake 1/2 speed");
            telemetry.addData("Outtake Power (2800 max): ", outtake.getVelocity());
            telemetry.addData("Intake Power (1.0 max): ", intake.getPower());
            telemetry.update();
        }
    }
}