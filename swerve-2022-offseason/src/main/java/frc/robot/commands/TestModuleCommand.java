package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Sensors.NavxGyro;
import frc.robot.subsystems.Swerve.SwerveModule;

public class TestModuleCommand extends CommandBase{
    private SwerveModule module;
    private DoubleSupplier x_axis;
    private DoubleSupplier y_axis;
    private NavxGyro gyro;
    public TestModuleCommand(SwerveModule module,DoubleSupplier x_axis, DoubleSupplier y_axis,NavxGyro gyro)
    {
        this.module = module;
        this.y_axis = y_axis;
        this.x_axis = x_axis;
        this.gyro = gyro;
        addRequirements(module);

    }
    @Override
    public void execute()
    {
        this.module.set_module_state(x_axis.getAsDouble()*46.5,y_axis.getAsDouble() * 180);
    }
    @Override
    public void end(boolean interupted)
    {
        this.module.set_module_state(0, gyro.getHeading().getDegrees());
    }
}
