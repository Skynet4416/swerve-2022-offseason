// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

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
  public SwerveModule(int top_can, int buttom_can) {
    this.top_motor = new CANSparkMax(top_can,MotorType.kBrushless);
    this.buttom_motor = new CANSparkMax(buttom_can,MotorType.kBrushless);
    this.top_encoder = this.top_motor.getEncoder();
    this.buttom_encoder = this.buttom_motor.getEncoder();
    this.top_controller = this.top_motor.getPIDController();
    this.buttom_controller = this.buttom_motor.getPIDController();
    this.angle_controller = new PIDController(0, 0, 0);
    this.velocity_controller = new PIDController(0,0,0);
    this.module_state = new SwerveModuleState(velocity_encoder,Rotation2d.fromDegrees(angle_encoder));
  }
  protected void set_velocity_buttom(double velocity){
    this.buttom_controller.setReference(velocity,ControlType.kVelocity);
  }
  protected void set_velocity_top(double velocity){
    this.top_controller.setReference(velocity,ControlType.kVelocity);
  }
  protected void set_precentage_buttom(double precentage){
    this.buttom_motor.set(precentage);
  }
  protected void set_precentage_top(double precentage){
    this.top_motor.set(precentage);
  }
  public void set_module_rotations(double velocity,double rotational_velocity)
  {
    this.set_velocity_buttom((velocity+rotational_velocity)/2);
    this.set_velocity_top((rotational_velocity-velocity)/2);
  }
  public void periodic() {
    this.velocity_encoder = this.top_encoder.getVelocity() - this.buttom_encoder.getVelocity();
    this.angle_encoder = this.top_encoder.getVelocity() + this.buttom_encoder.getVelocity();
  }
  public void set_module_state(double target_velcity)
  {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
