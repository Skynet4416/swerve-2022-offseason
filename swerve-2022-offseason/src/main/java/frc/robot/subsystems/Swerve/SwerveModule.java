// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.time.Duration;
import java.time.Instant;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax top_motor;
  private CANSparkMax buttom_motor;
  private RelativeEncoder top_encoder;
  private RelativeEncoder buttom_encoder;
  public SwerveModuleState module_state;
  private SparkMaxPIDController top_controller;
  private SparkMaxPIDController buttom_controller;
  private PIDController velocity_controller;
  private double velocity_encoder;
  private double angle_encoder;
  private PIDController angle_controller;
  private Instant start_time;
  private Instant end_time;
  private double wheel_radius;

  public SwerveModule(int top_can, int buttom_can, double wheel_radius) {
    this.top_motor = new CANSparkMax(top_can, MotorType.kBrushless);
    this.buttom_motor = new CANSparkMax(buttom_can, MotorType.kBrushless);
    this.top_encoder = this.top_motor.getEncoder();
    this.top_encoder.setVelocityConversionFactor(wheel_radius * 2 * Math.PI / 60);
    this.buttom_encoder.setVelocityConversionFactor(wheel_radius * 2 * Math.PI / 60);
    this.buttom_encoder = this.buttom_motor.getEncoder();
    this.top_controller = this.top_motor.getPIDController();
    this.buttom_controller = this.buttom_motor.getPIDController();
    this.angle_controller = new PIDController(0, 0, 0);
    this.velocity_controller = new PIDController(0, 0, 0);
    this.module_state = new SwerveModuleState(velocity_encoder, Rotation2d.fromDegrees(angle_encoder));
    this.start_time = Instant.now();
    this.wheel_radius = wheel_radius;
  }

  protected void set_velocity_buttom(double velocity) {
    this.buttom_controller.setReference(velocity, ControlType.kVelocity);
  }

  protected void set_velocity_top(double velocity) {
    this.top_controller.setReference(velocity, ControlType.kVelocity);
  }

  protected void set_precentage_buttom(double precentage) {
    this.buttom_motor.set(precentage);
  }

  protected void set_precentage_top(double precentage) {
    this.top_motor.set(precentage);
  }

  public void set_module_rotations(double velocity, double rotational_velocity) {
    this.set_velocity_buttom((velocity + rotational_velocity) / 2);
    this.set_velocity_top((rotational_velocity - velocity) / 2);
  }

  @Override
  public void periodic() {
    this.end_time = Instant.now();
    this.velocity_encoder = this.top_encoder.getVelocity() - this.buttom_encoder.getVelocity();
    this.angle_encoder = this.top_encoder.getVelocity() + this.buttom_encoder.getVelocity();
    this.module_state.angle = Rotation2d.fromDegrees(this.module_state.angle.getDegrees()
        + angle_encoder * (Duration.between(end_time, start_time).toMillis() / 1000));
    this.module_state.speedMetersPerSecond = wheel_radius * 2 * Math.PI * velocity_encoder / 60;
    this.start_time = Instant.now();
  }

  public void set_module_state(double target_velcity, double target_rotation) {
    this.set_module_rotations(this.velocity_controller.calculate(this.velocity_encoder, target_velcity),
        this.angle_controller.calculate(this.module_state.angle.getDegrees(), target_rotation));
  }

  public void set_module_state(SwerveModuleState target_state) {
    this.set_module_rotations(
        this.velocity_controller.calculate(this.velocity_encoder, target_state.speedMetersPerSecond),
        this.angle_controller.calculate(this.module_state.angle.getDegrees(), target_state.angle.getDegrees()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
