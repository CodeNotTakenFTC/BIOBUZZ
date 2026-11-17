package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name="Auto_V1", group="Odometry Auto")
public class Auto_V1 extends LinearOpMode {

    private final double outtakePower = 0.62;
    private final int chargeTime = 5500;
    private final int launchTime = 1200;

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
        sleep(chargeTime);
        intake.intake();

        // shoot preload and tip #1
        launchSequence(drive, turret, outtake);

        // back into garden, collect pollen, drive to opposite flower
        Action path1 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(66, -55, Math.toRadians(90)), Math.toRadians(270))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(24, -48, Math.toRadians(45)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-24, -42, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-54, -24, Math.toRadians(0)), Math.toRadians(180))
                .build();
        Actions.runBlocking(driveAndAim(path1, drive, turret));

        // shoot 4 pollen
        aimTurretAndWait(drive, turret);
        outtake.openFeedServo();
        sleep(launchTime);
        outtake.closeFeedServo();

        //intake.scoopDown(); TODO: add to intake subsystem

        // collect 4 pollen from flower
        Action path2 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(-60, -24, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(1.0)
                .splineToLinearHeading(new Pose2d(-54, -24, Math.toRadians(0)), Math.toRadians(0))
                .build();
        Actions.runBlocking(driveAndAim(path2, drive, turret));

        // shoot 4 pollen and tip #2
        launchSequence(drive, turret, outtake);

        // park
        Action path3 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(-42, -60, Math.toRadians(270)), Math.toRadians(270))
                .build();
        Actions.runBlocking(driveAndAim(path3, drive, turret));

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
