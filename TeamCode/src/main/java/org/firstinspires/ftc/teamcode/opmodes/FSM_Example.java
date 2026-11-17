
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "BIOBUZZ FSM Example")
public class FSM_Example extends LinearOpMode {

    // The states our robot can be in
    enum State {
        DRIVE_TO_POLLEN,
        COLLECT_POLLEN,
        DRIVE_TO_HIVE,
        CHECK_HIVE,
        SCORE,
        DONE
    }

    State state = State.DRIVE_TO_POLLEN;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // TODO: Initialize robot, intake, outtake, Pinpoint

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && state != State.DONE) {

            switch (state) {

                case DRIVE_TO_POLLEN:
                    telemetry.addLine("Driving to pollen");

                    // TODO: Start Road Runner path
                    // TODO: Check if path is finished

                    if (driveToPollen()) {
                        state = State.COLLECT_POLLEN;
                        timer.reset();
                    }
                    break;


                case COLLECT_POLLEN:
                    telemetry.addLine("Collecting pollen");

                    // TODO: Turn on intake

                    if (pollenCollected() || timer.seconds() > 3) {
                        // TODO: Turn off intake
                        state = State.DRIVE_TO_HIVE;
                    }
                    break;


                case DRIVE_TO_HIVE:
                    telemetry.addLine("Driving to HIVE");

                    // TODO: Start Road Runner path to scoring position

                    if (driveToHive()) {
                        state = State.CHECK_HIVE;
                    }
                    break;


                case CHECK_HIVE:
                    telemetry.addLine("Checking HIVE");

                    // TODO: Use Limelight to check HIVE state

                    if (hiveReady()) {
                        state = State.SCORE;
                    } else {
                        // HIVE not ready, check again
                        state = State.CHECK_HIVE;
                    }
                    break;


                case SCORE:
                    telemetry.addLine("Scoring pollen");

                    // TODO: Aim turret
                    // TODO: Set outtake velocity
                    // TODO: Feed pollen

                    if (scoreComplete()) {
                        state = State.DONE;
                    }
                    break;


                case DONE:
                    telemetry.addLine("Autonomous complete");
                    break;
            }

            telemetry.addData("Current State", state);
            telemetry.update();
        }

        // TODO: Stop motors
    }


    // Placeholder methods - connect these to your robot code

    private boolean driveToPollen() {
        return true; // Replace with Road Runner completion check
    }

    private boolean pollenCollected() {
        return false; // Replace with sensor or timed intake logic
    }

    private boolean driveToHive() {
        return true; // Replace with Road Runner completion check
    }

    private boolean hiveReady() {
        return true; // Replace with Limelight detection
    }

    private boolean scoreComplete() {
        return true; // Replace with scoring completion logic
    }
}

