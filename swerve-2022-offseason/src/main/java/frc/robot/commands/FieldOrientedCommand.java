// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve.SwerveSubsytem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FieldOrientedCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private SwerveSubsytem swerve_subsytem;
  private DoubleSupplier x_velocity,y_velocity,rotation_velocity;
  private double max_velocity;
  private double max_rotation;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FieldOrientedCommand(SwerveSubsytem swerve_subsytem,DoubleSupplier x_velocity, DoubleSupplier y_velocity, DoubleSupplier rotation_velocity ) {
    this.swerve_subsytem = swerve_subsytem;
    this.x_velocity = x_velocity;
    this.y_velocity = y_velocity;
    this.rotation_velocity = rotation_velocity;
    this.max_velocity = 5;
    this.max_rotation = Math.PI;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve_subsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.swerve_subsytem.set_modules_field_orianted(x_velocity.getAsDouble()*max_velocity, y_velocity.getAsDouble()*max_velocity, rotation_velocity.getAsDouble()*max_rotation);
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
