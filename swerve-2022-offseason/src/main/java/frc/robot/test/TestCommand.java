package frc.robot.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestCommand extends CommandBase{
    private TestSubsytem subsytem;
    public TestCommand(TestSubsytem subsytem)
    {
        this.subsytem = subsytem;
    }
    @Override
    public void initialize()
    {
        this.subsytem.setRPM(SmartDashboard.getNumber("TEST RPM SETPOINT", 0));
    }
    @Override
    public void end(boolean interupted)
    {
        
        this.subsytem.setRPM(0);
    
    }
}
