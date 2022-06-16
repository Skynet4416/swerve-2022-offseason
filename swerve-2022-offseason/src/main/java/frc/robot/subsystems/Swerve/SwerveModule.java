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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Meth.MyPIDController;

public class SwerveModule extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax top_motor;
  private CANSparkMax buttom_motor;
  private RelativeEncoder top_encoder; // RPM
  private RelativeEncoder buttom_encoder; // RPM
  public SwerveModuleState module_state;
  private SparkMaxPIDController top_controller; 
  private SparkMaxPIDController buttom_controller;
  private MyPIDController velocity_controller; //RPM wheel velocity
  private double wheel_rpm_encoder; // RPM wheel velocity
  private double wheel_angle_velocity_rpm; // rpm angle wheel
  private MyPIDController angle_controller; // deg wheel
  private Instant start_time;
  private Instant end_time;
  private double wheel_radius; // m
  private String name;

  public SwerveModule(int top_can, int buttom_can, double wheel_radius, String name){

    this.name = name;
    this.top_motor = new CANSparkMax(top_can, MotorType.kBrushless);
    this.buttom_motor = new CANSparkMax(buttom_can, MotorType.kBrushless);
    this.top_motor.setIdleMode(IdleMode.kBrake);
    this.buttom_motor.setIdleMode(IdleMode.kBrake);
    this.top_encoder = this.top_motor.getEncoder();
    this.buttom_encoder = this.buttom_motor.getEncoder();
    this.top_controller = this.top_motor.getPIDController();
    this.buttom_controller = this.buttom_motor.getPIDController();
    this.angle_controller = new MyPIDController(0, 0, 0, 0);
    this.velocity_controller = new MyPIDController(0, 0, 0, 0);
    this.module_state = new SwerveModuleState(wheel_rpm_encoder, Rotation2d.fromDegrees(0));
    this.start_time = Instant.now();
    this.wheel_radius = wheel_radius;
  }

  public double get_top_mps()
  {
    return convert_rpm_to_mps(top_encoder.getVelocity());
  }
  public double get_buttom_mps()
  {
    return convert_rpm_to_mps(buttom_encoder.getVelocity());
  }
  public double convert_rpm_to_mps(double rpm)
  {
    return wheel_radius* 2*Math.PI * rpm / 60;
  }

  public double convert_mps_to_rpm(double mps)
  {
    return 60 * mps / (2 * Math.PI * wheel_radius);
  }
  protected void set_rpm_buttom(double target_rpm) {
    this.buttom_controller.setReference(target_rpm, ControlType.kVelocity);
    SmartDashboard.putNumber(name + " buttom RPM setpoint", target_rpm);
  }

  protected void set_rpm_top(double target_rpm) {
    this.top_controller.setReference(target_rpm, ControlType.kVelocity);
    SmartDashboard.putNumber(name + " top setpoint", target_rpm);

  }

  protected void set_precentage_buttom(double precentage) {
    this.buttom_motor.set(precentage);
  }

  protected void set_precentage_top(double precentage) {
    this.top_motor.set(precentage);
  }

  public void set_module_rotations(double wanted_wheel_rpm, double rotational_velocity_rpm) {
    /**
     * wanted_wheel_rpm - the wanted wheel rpm
     * rotational_velocity - the rotational velcoity rpm
     */
    this.set_rpm_buttom((wanted_wheel_rpm + rotational_velocity_rpm) / 2);
    this.set_rpm_top((rotational_velocity_rpm - wanted_wheel_rpm) / 2);
  }

  @Override
  public void periodic() {
    this.end_time = Instant.now();
    this.wheel_rpm_encoder = this.top_encoder.getVelocity() - this.buttom_encoder.getVelocity();
    this.wheel_angle_velocity_rpm = this.top_encoder.getVelocity() + this.buttom_encoder.getVelocity();
    this.module_state.angle = Rotation2d.fromDegrees((this.module_state.angle.getDegrees()
        + wheel_angle_velocity_rpm * (Duration.between(end_time, start_time).toMillis() / 1000)) % 360);
    this.module_state.speedMetersPerSecond = convert_rpm_to_mps(this.wheel_rpm_encoder);
    this.start_time = Instant.now();
    this.update_pid();
    SmartDashboard.putNumber(name + " wheel velcoity RPM", wheel_rpm_encoder);
    SmartDashboard.putNumber(name + " angle velocity RPM", wheel_angle_velocity_rpm);
    SmartDashboard.putNumber(name + " buttom RPM", buttom_encoder.getVelocity());
    SmartDashboard.putNumber(name + " top RPM", top_encoder.getVelocity());
    SmartDashboard.putNumber(name + " RPM setpoint",velocity_controller.getSetpoint());
    SmartDashboard.putNumber(name + " angle setpoint deg", angle_controller.getSetpoint());
    SmartDashboard.putNumber(name + " angle deg", this.module_state.angle.getDegrees());

  }

  protected void update_pid() {
    this.top_controller.setD(SmartDashboard.getNumber(name + " top kd", 0));
    this.top_controller.setI(SmartDashboard.getNumber(name + " top ki", 0));
    this.top_controller.setP(SmartDashboard.getNumber(name + " top kp", 0));
    this.top_controller.setFF(SmartDashboard.getNumber(name + " top kff", 0));
    this.buttom_controller.setD(SmartDashboard.getNumber(name + " buttom kd", 0));
    this.buttom_controller.setI(SmartDashboard.getNumber(name + " buttom ki", 0));
    this.buttom_controller.setP(SmartDashboard.getNumber(name + " buttom kp", 0));
    this.buttom_controller.setFF(SmartDashboard.getNumber(name + " buttom kff", 0));
    this.angle_controller.setD(SmartDashboard.getNumber(name + " wheel angle kd", 0));
    this.angle_controller.setI(SmartDashboard.getNumber(name + " wheel angle ki", 0));
    this.angle_controller.setP(SmartDashboard.getNumber(name + " wheel angle kp", 0));
    this.angle_controller.setFF(SmartDashboard.getNumber(name + " wheel angle kff", 0));
    this.velocity_controller.setD(SmartDashboard.getNumber(name + " wheel velocity kd", 0));
    this.velocity_controller.setI(SmartDashboard.getNumber(name + " wheel velocity ki", 0));
    this.velocity_controller.setP(SmartDashboard.getNumber(name + " wheel velocity kp", 0));
    this.velocity_controller.setFF(SmartDashboard.getNumber(name + " wheel velocity kff", 0));
  }

  public void set_module_state(double target_velocity_in_mps, double target_rotation_in_deg) {
    this.set_module_rotations(this.velocity_controller.calculate(this.wheel_rpm_encoder, convert_mps_to_rpm(target_velocity_in_mps)),
        this.angle_controller.calculate(this.module_state.angle.getDegrees(), target_rotation_in_deg));
  }

  public void set_module_state(SwerveModuleState target_state) {
    this.set_module_rotations(
        this.velocity_controller.calculate(this.wheel_rpm_encoder, convert_mps_to_rpm(target_state.speedMetersPerSecond)),
        this.angle_controller.calculate(this.module_state.angle.getDegrees(), target_state.angle.getDegrees()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
