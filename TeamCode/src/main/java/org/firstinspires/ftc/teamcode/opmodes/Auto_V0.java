package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Auto_V0", group="Odometry Auto")
public class Auto_V0 extends LinearOpMode {

    // TODO shooter and PIDF variables/coefficients
    // 6000 RPM / 60 SEC/MIN * 28 TICKS/REV
    private static double MAX_TICKS = 6000.0 / 60.0 * 28.0;


    // TODO other parameters (outtakePower, chargeTime, transferTime, transferPower, waitTime, etc.
    private double outtakePower = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "transfer");
        DcMotorEx outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        DcMotorEx outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");

        // TODO set reverse and forward motors, brake, set PIDF coefficients
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //outtakeLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        //outtakeRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        waitForStart();

        // ------------------------- //
        // -- START AUTO SEQUENCE -- //
        // ------------------------- //

       // TODO create starting pose
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // TODO set motors at start of auto
        double targetVelocity = outtakePower * MAX_TICKS;
        outtakeLeft.setVelocity(targetVelocity);
        outtakeRight.setVelocity(targetVelocity);
        //sleep(chargeTime);

        intake.setPower(1.0);

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(0.0)
                .splineToLinearHeading(new Pose2d(12, 12, Math.toRadians(0)), Math.toRadians(90))
                .strafeTo(new Vector2d(36, -66))
                .build()
        );
    }
}
