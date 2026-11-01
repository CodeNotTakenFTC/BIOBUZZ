package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Auto_V0", group="Odometry Auto")
public class Auto_V0 extends LinearOpMode {

    // TODO shooter and PIDF variables/coefficients
    // 6000 RPM / 60 SEC/MIN * 28 TICKS/REV
    public static double MAX_TICKS = 6000.0 / 60.0 * 28.0;

    // TODO other parameters (outtakePower, chargeTime, waitTime, etc.
    private double outtakePower = 0.40;
    private int chargeTime = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "outtake");

        // TODO set reverse and forward motors, brake, set PIDF coefficients
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(18, 0, 0, 13.5));

        waitForStart();

        // ------------------------- //
        // -- START AUTO SEQUENCE -- //
        // ------------------------- //

       // TODO create starting pose
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // TODO set motors at start of auto
        outtake.setVelocity(MAX_TICKS * outtakePower);
        sleep(chargeTime);

        intake.setPower(1.0);

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(12, 12, Math.toRadians(0)), Math.toRadians(90))
                .strafeTo(new Vector2d(36, -66))
                .build()
        );
    }
}
