package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name="Auto_V0", group="Odometry Auto")
public class Auto_V0 extends LinearOpMode {

    // TODO other parameters (outtakePower, chargeTime, waitTime, etc.
    private double outtakePower = 0.40;
    private int chargeTime = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        Pose2d beginPose = new Pose2d(60, -20, Math.toRadians(90));    // must match end of AUTO
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, true);

        waitForStart();

        // ------------------------- //
        // -- START AUTO SEQUENCE -- //
        // ------------------------- //

        // SHOOTING SEQUENCE TODO put in a helper method
        outtake.setRPM(6000 * outtakePower);
        intake.intake();
        sleep(chargeTime);
        outtake.openFeedServo();

        if (isStopRequested()) return;

        Action path1 = drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(60, -45, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(24, -48, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(60, -20, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Actions.runBlocking(driveAndAim(path1, drive, turret));

        aimTurretAndWait(drive, turret);
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
