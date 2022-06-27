package frc.robot.test;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsytem extends SubsystemBase {
    private CANSparkMax motor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;

    public TestSubsytem() {
        this.motor = new CANSparkMax(10, MotorType.kBrushless);
        this.pidController = this.motor.getPIDController();
        this.encoder = this.motor.getEncoder();
        pidController.setD(0);
        pidController.setFF(0);
        pidController.setI(0);
    }

    public void setRPM(double RPM) {
        this.pidController.setReference(RPM, ControlType.kVelocity);
        SmartDashboard.putNumber("TEST RPM SETPOINT", RPM);
    }
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("TEST RPM", encoder.getVelocity());
        pidController.setD(SmartDashboard.getNumber("TEST kd", 0));
        pidController.setFF(SmartDashboard.getNumber("TEST kff", 0));
        pidController.setI(SmartDashboard.getNumber("TEST ki", 0));
        pidController.setP(SmartDashboard.getNumber("TEST kp", 0));
    }

}
