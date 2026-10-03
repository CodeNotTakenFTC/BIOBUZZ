package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name="Auto_FAR_BLUE_rr", group="Odometry Auto")
public class Auto_FAR_BLUE_rr extends LinearOpMode {

    // motor declarations
    private DcMotor intake, belt;
    private DcMotorEx outtakeLeft, outtakeRight;        // allows velocity control

    // velocity based shooting and PIDF coefficients
    public static double maxTicks = 6000.0 / 60.0 * 28.0;   //based on 6000rpm motor
    public static double P = 18.0; //tuned with FTC Dashboard for fast steady spinup
    public static double I = 0.0;  //stays zero
    public static double D = 0.0;  //stays zero
    public static double F = 13.0; //tuned with FTC Dashboard for fast recovery

    // launch power and timing parameters
    private double outtakePwr = 0.50355;   // Outtake power level (adjust for distance) 0.55 long range, 0.45? close
    private int chargeTime = 2000;     // time to initially charge outtake - 2750 good (optimize)
    private int beltLaunchTime = 1650;   // how long to run belt for launching - 750 good (optimize)
    private double beltPower = 0.50;    // how fast to run belt for shooting
    private double waitTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // init intake, outtake, belt motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outRight");
        belt = hardwareMap.get(DcMotor.class, "belt");

        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        outtakeRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        waitForStart();

        // ------------------------- //
        // -- START AUTO SEQUENCE -- //
        // gather and shoot 15 balls //
        // ------------------------- //

        // create starting pose
        Pose2d beginPose = new Pose2d(60, -16, Math.toRadians(180)); // touching back wall with robot/armor edge above far seem
        // create RR drive obj
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        double targetVelocity = outtakePwr * maxTicks;
        outtakeLeft.setVelocity(targetVelocity);            // keep outtake spun up entire AUTO
        outtakeRight.setVelocity(targetVelocity);
        sleep(chargeTime);

        intake.setPower(1.0);     // keep intake on entire auto

        // TODO: still seems to drag ground occasionally after gathering middle and sometimes after gathering near

        // --- FIRST 3 PRELOAD LAUNCH --- //
        Action path1 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(54, -16, Math.toRadians(205)), Math.toRadians(205))
                .build();
        Actions.runBlocking(path1);

        // shoot first 3 balls 1-3
        launchSequence();

        // --- FAR 3 GATHER AND LAUNCH --- //
        Action path2 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(36, -20, Math.toRadians(90)), Math.toRadians(205))
                .strafeTo(new Vector2d(36, -66))
                .build();
        Actions.runBlocking(path2);

        beltLoad();

        Action path9 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(waitTime)
                .splineToLinearHeading(new Pose2d(54, -16, Math.toRadians(203)), Math.toRadians(45), null, new ProfileAccelConstraint(-30, 30))
                .build();
        Actions.runBlocking(path9);

        // shoot next 3 balls 4-6
        launchSequence();

        // --- MIDDLE 3 GATHER AND LAUNCH --- //
        Action path3 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(12, -20, Math.toRadians(90)), Math.toRadians(205))
                .strafeTo(new Vector2d(12, -66))
                .build();
        Actions.runBlocking(path3);

        beltLoad();

        Action path10 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(waitTime)
                //.strafeTo(new Vector2d(12, -40))
                .splineToLinearHeading(new Pose2d(54, -16, Math.toRadians(203)), Math.toRadians(45), null, new ProfileAccelConstraint(-35, 35))
                .build();
        Actions.runBlocking(path10);

        // shoot next 3 balls 7-9
        launchSequence();

        // --- NEAR 3 GATHER AND LAUNCH --- //
        Action path4 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(-12, -20, Math.toRadians(90)), Math.toRadians(205))
                .strafeTo(new Vector2d(-12, -58))
                .build();
        Actions.runBlocking(path4);

        beltLoad();

        Action path8 = drive.actionBuilder(drive.localizer.getPose())
                .waitSeconds(waitTime)
                .splineToLinearHeading(new Pose2d(54, -16, Math.toRadians(203)), Math.toRadians(45), null, new ProfileAccelConstraint(-35, 35))
                .build();
        Actions.runBlocking(path8);

        // shoot next 3 balls 10-12
        launchSequence();

        /*
        // --- LOADING ZONE 3 GATHER AND LAUNCH --- //
        Action path5 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(40, -60, Math.toRadians(180)), Math.toRadians(215))
                .strafeTo(new Vector2d(60, -60))
                .waitSeconds(waitTime)
                .splineToLinearHeading(new Pose2d(54, -16, Math.toRadians(205)), Math.toRadians(0))
                .build();
        Actions.runBlocking(path5);

        // shoot last 3 balls 13-15
        launchSequence();
        */

        // --- LEAVE LAUNCH ZONE AND PARK BY GATE --- //
        Action path6 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(90)), Math.toRadians(270))
                .build();
        Actions.runBlocking(path6);
    }

    public void launchSequence() {
        belt.setPower(-1);              // clear wheel
        sleep(50);
        belt.setPower(0);

        belt.setPower(beltPower);         // advance balls through shooter
        sleep(beltLaunchTime);
        belt.setPower(0);
    }

    public void beltLoad() {
        belt.setPower(1);
        sleep(100);
        belt.setPower(0);
    }
}