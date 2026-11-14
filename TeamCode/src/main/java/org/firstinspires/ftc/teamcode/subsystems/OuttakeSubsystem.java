package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeSubsystem {

    // Hardware
    private DcMotorEx outtakeMotor;
    private Servo feedServo;

    // Motor settings
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000.0;

    // Servo positions
    private static final double SERVO_OPEN = 0.3;
    private static final double SERVO_CLOSED = 0.0;

    public OuttakeSubsystem(HardwareMap hardwareMap) {

        // Map hardware
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        feedServo = hardwareMap.get(Servo.class, "servo1");

        // Motor configuration
        outtakeMotor.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Your existing velocity PIDF values
        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(18, 0, 0, 13.5));

        // Initial positions
        outtakeMotor.setVelocity(0);
        feedServo.setPosition(SERVO_CLOSED);
    }

    // Set outtake speed in RPM
    public void setRPM(double rpm) {
        double ticksPerSecond = rpm * TICKS_PER_REV / 60.0;
        outtakeMotor.setVelocity(ticksPerSecond);
    }

    // Stop outtake motor
    public void stop() {
        outtakeMotor.setVelocity(0);
    }

    public void openFeedServo() {
        feedServo.setPosition(SERVO_OPEN);
    }

    public void closeFeedServo() {
        feedServo.setPosition(SERVO_CLOSED);
    }

    // Return current motor velocity for telemetry
    public double getVelocity() {
        return outtakeMotor.getVelocity();
    }
}