// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve.SwerveSubsytem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FieldOrientedCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private SwerveSubsytem swerve_subsytem;
  private DoubleSupplier x_velocity, y_velocity, rotation_velocity;
  private double max_velocity;
  private double max_rotation;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FieldOrientedCommand(SwerveSubsytem swerve_subsytem, DoubleSupplier x_velocity, DoubleSupplier y_velocity,
      DoubleSupplier rotation_velocity) {
    this.swerve_subsytem = swerve_subsytem;
    this.x_velocity = x_velocity;
    this.y_velocity = y_velocity;
    this.rotation_velocity = rotation_velocity;
    this.max_velocity = 4;
    this.max_rotation = Math.PI;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve_subsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double corrected_x_velocity = x_velocity.getAsDouble();
    double corrected_y_velocity = y_velocity.getAsDouble();
    double corrected_rotation_velocity = rotation_velocity.getAsDouble();

    if (Math.abs(corrected_x_velocity) < 0.2)
      corrected_x_velocity = 0;
    if (Math.abs(corrected_y_velocity) < 0.2)
      corrected_y_velocity = 0;
    if (Math.abs(corrected_rotation_velocity) < 0.2)
      corrected_rotation_velocity = 0;
    this.swerve_subsytem.set_modules_field_orianted(corrected_x_velocity * max_velocity,
        corrected_y_velocity * max_velocity, corrected_rotation_velocity * max_rotation);
    SmartDashboard.putNumber("left x", corrected_x_velocity);
    SmartDashboard.putNumber("left y", corrected_y_velocity);
    SmartDashboard.putNumber("right x",corrected_rotation_velocity);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.swerve_subsytem.set_modules_field_orianted(0, 0, rotation_velocity.getAsDouble());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
