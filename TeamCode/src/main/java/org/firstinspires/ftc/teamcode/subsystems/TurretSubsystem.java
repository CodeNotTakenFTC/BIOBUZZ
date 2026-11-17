package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretSubsystem {

    private final DcMotorEx turretMotor;
    private boolean manualDriving = false;

    // ==========================================
    // FIELD TARGET (inches)
    // ==========================================
    private static final double TARGET_X = 0.0;
    private static final double TARGET_Y = -12.0;    //-12 RED, 12 BLUE

    // ==========================================
    // MOTOR / GEARING
    // ==========================================
    private static final double TURRET_GEAR_RATIO = 2.75;
    private static final double MOTOR_OUTPUT_TICKS_PER_REV = 384.5;
    private static final double TICKS_PER_DEGREE = (MOTOR_OUTPUT_TICKS_PER_REV * TURRET_GEAR_RATIO) / 360.0;

    // ==========================================
    // TURRET LIMITS
    // ==========================================
    private static final double MIN_ANGLE = -90.0;
    private static final double MAX_ANGLE = 170.0;
    private static final double MAX_POWER = 0.8;

    // ==========================================
    // CONSTRUCTOR
    // ==========================================
    public TurretSubsystem(HardwareMap hardwareMap, boolean resetEncoder) {

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (resetEncoder) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0);
    }

    // ==========================================
    // UPDATE TURRET AIM
    // ==========================================

    public void update(Pose2d robotPose) {

        // Difference between target and robot
        double deltaX = TARGET_X - robotPose.position.x;
        double deltaY = TARGET_Y - robotPose.position.y;

        // Field angle toward target
        double targetAngle = Math.atan2(deltaY, deltaX);

        // Convert to robot-relative angle
        double turretAngle = targetAngle - robotPose.heading.toDouble();

        // Normalize angle
        turretAngle = Math.atan2(Math.sin(turretAngle), Math.cos(turretAngle));

        // Convert radians to degrees
        double angleDegrees = Math.toDegrees(turretAngle);

        // Protect turret wiring
        angleDegrees = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDegrees));

        // Convert degrees to encoder ticks
        int targetTicks = (int) (angleDegrees * TICKS_PER_DEGREE);

        // Command motor
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setPower(MAX_POWER);
    }

    public void manualControl(double stickInput) {

        // Deadzone prevents accidental movement.
        if (Math.abs(stickInput) > 0.08) {

            // Switch to manual motor control when needed.
            if (!manualDriving) {
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                manualDriving = true;
            }

            // Adjust power to control turret speed.
            turretMotor.setPower(-stickInput * 0.3);

        } else {

            // When the stick is released, capture the current position.
            if (manualDriving) {
                turretMotor.setPower(0);

                turretMotor.setTargetPosition(turretMotor.getCurrentPosition());

                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(0.3);

                manualDriving = false;
            }
        }
    }

    // ==========================================
    // TELEMETRY HELPERS
    // ==========================================

    public int getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return turretMotor.getTargetPosition();
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}