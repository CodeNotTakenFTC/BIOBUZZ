package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Tag Test", group = "Test")
public class LimeLight_Tag_Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Get the Limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        // Start the Limelight
        limelight.start();

        telemetry.addLine("Limelight started");
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Get the latest result from the Limelight
            LLResult result = limelight.getLatestResult();

            if (result == null) {
                telemetry.addLine("NO RESULT");
            } else {
                telemetry.addData("Result Valid", result.isValid());

                // Get all AprilTags detected
                for (LLResultTypes.FiducialResult tag :
                        result.getFiducialResults()) {

                    telemetry.addData("TAG ID", tag.getFiducialId());
                    telemetry.addData("X Angle", "%.2f degrees", tag.getTargetXDegrees());
                    telemetry.addData("Y Angle", "%.2f degrees", tag.getTargetYDegrees());
                    telemetry.addLine("----------------");
                }

                // No AprilTags found
                if (result.getFiducialResults().isEmpty()) {
                    telemetry.addLine("NO APRILTAGS DETECTED");
                }
            }

            telemetry.update();

            // Don't hammer the Limelight
            sleep(20);
        }

        limelight.stop();
    }
}
