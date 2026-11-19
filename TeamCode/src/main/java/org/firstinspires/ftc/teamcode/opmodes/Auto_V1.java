package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name="Auto_V1", group="Odometry Auto")
public class Auto_V1 extends LinearOpMode {

    private final double outtakePower = 0.56;  //0.56 fresh battery
    private final int chargeTime = 5500;
    private final int launchTime = 1200;
    private final double waitTime = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        Pose2d beginPose = new Pose2d(66, -16, Math.toRadians(90)); // STARTING LOCATION ON FIELD
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, true);

        waitForStart();

        // ------------------------- //
        // -- START AUTO SEQUENCE -- //
        // ------------------------- //

        // charging sequence
        outtake.setRPM(6000 * outtakePower);
        aimTurretAndWait(drive, turret);
        sleep(chargeTime);
        intake.intake();

        // shoot preload and tip #1
        launchSequence(drive, turret, outtake);

        // back into garden, collect pollen, drive back to starting location
        Action path1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(66, -64))
                .strafeTo(new Vector2d(66, -16))
                .build();
        Actions.runBlocking(driveAndAim(path1, drive, turret));

        // shoot pollen guarantee tip #1
        launchSequence(drive, turret, outtake);

        // drive to in front of near flower
        Action path2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(24, -48))
                .build();
        Actions.runBlocking(driveAndAim(path2, drive, turret));

        intake.flowerDown();
        sleep(200);

        // collect 4 pollen from close flower
        Action path22 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(24, -60))
                .waitSeconds(waitTime)
                .strafeTo(new Vector2d(24, -40))
                .build();
        Actions.runBlocking(driveAndAim(path22, drive, turret));

        intake.flowerUp();

        // go to opposite launch zone
        Action path3 = drive.actionBuilder(drive.localizer.getPose())
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(-42, -30))
                .strafeTo(new Vector2d(-42, -16))
                .build();
        Actions.runBlocking(driveAndAim(path3, drive, turret));

        // shoot 4 pollen
        launchSequence(drive, turret, outtake);

        // position in front of flower
        Action path33 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-42, -16))
                .build();
        Actions.runBlocking(driveAndAim(path33, drive, turret));

        intake.flowerDown();
        sleep(200);

        // collect 4 pollen from far flower
        Action path4 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-54, -16))
                .waitSeconds(waitTime)
                .strafeTo(new Vector2d(-45, -16))
                .build();
        Actions.runBlocking(driveAndAim(path4, drive, turret));

        intake.flowerUp();

        // shoot 4 pollen and tip #2
        launchSequence(drive, turret, outtake);

        // park
        Action path5 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-40, -52)) // TODO: enter into MAIN
                .build();
        Actions.runBlocking(driveAndAim(path5, drive, turret));
    }

    private void launchSequence(MecanumDrive drive, TurretSubsystem turret, OuttakeSubsystem outtake) {
        aimTurretAndWait(drive, turret);
        outtake.openFeedServo();
        sleep(launchTime);
        outtake.closeFeedServo();
    }

    private void aimTurretAndWait(MecanumDrive drive, TurretSubsystem turret) {
        // Get current robot position
        drive.updatePoseEstimate();
        Pose2d robotPose = drive.localizer.getPose();

        // Command turret to aim at scoring target
        turret.update(robotPose);

        // Wait until turret reaches target
        while (opModeIsActive() && Math.abs(turret.getTargetPosition()- turret.getCurrentPosition()) > 10) {
            idle();
        }
    }

    private Action driveAndAim(Action driveAction, MecanumDrive drive, TurretSubsystem turret) {

        return new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {

                // Let Road Runner execute its next step.
                boolean pathRunning = driveAction.run(packet);

                // Get the latest robot position from Pinpoint.
                Pose2d robotPose = drive.localizer.getPose();

                // Keep the turret aimed at the field target.
                turret.update(robotPose);

                return pathRunning;
            }
        };
    }
}
