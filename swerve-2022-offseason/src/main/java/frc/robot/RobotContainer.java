// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Sensors.NavxGyro;
import frc.robot.commands.FieldOrientedCommand;
import frc.robot.commands.TestModuleCommand;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve.SwerveSubsytem;
import frc.robot.test.TestCommand;
import frc.robot.test.TestSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private NavxGyro gyro = new NavxGyro(Port.kMXP);
  public SwerveSubsytem swerve_subsytem = new SwerveSubsytem(gyro);
  private XboxController xbox_controller = new XboxController(0);

  private FieldOrientedCommand FOFF = new FieldOrientedCommand(swerve_subsytem, () -> xbox_controller.getLeftY(),
      () -> xbox_controller.getLeftX(), () -> xbox_controller.getRightX());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    swerve_subsytem.setDefaultCommand(FOFF);
    LiveWindow.disableAllTelemetry();
    // A.whileHeld(command);
    // SmartDashboard.putNumber("TEST kp", 0);
    // SmartDashboard.putNumber("TEST kff", 0);
    // SmartDashboard.putNumber("TEST ki", 0);
    // SmartDashboard.putNumber("TEST kd", 0);
    // SmartDashboard.putNumber("TEST RPM SETPOINT", 0);
    // swerve_module.setDefaultCommand(new TestModuleCommand(swerve_module, ()-> xbox_controller.getLeftX(), ()-> xbox_controller.getRightX(), gyro));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
