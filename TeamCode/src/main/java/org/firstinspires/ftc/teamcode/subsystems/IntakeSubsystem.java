
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    private DcMotor intakeMotor, intakeMotor2;
    private Servo leftFlower, rightFlower;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "transfer");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFlower = hardwareMap.get(Servo.class, "leftflower");
        rightFlower = hardwareMap.get(Servo.class, "rightflower");
    }

    // Run intake forward
    public void intake() {
        intakeMotor.setPower(1.0);
        intakeMotor2.setPower(1.0);
    }

    // Run intake backward
    public void reverse() {
        intakeMotor.setPower(-1.0);
        intakeMotor2.setPower(-1.0);
    }

    // Stop intake
    public void stop() {
        intakeMotor.setPower(0);
        intakeMotor2.setPower(0);
    }

    public void flowerDown() {
        leftFlower.setPosition(0.5485);
        rightFlower.setPosition(0.4515);
    }

    public void flowerUp() {
        leftFlower.setPosition(1);
        rightFlower.setPosition(0);
    }

    // Set a custom power
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }
}
