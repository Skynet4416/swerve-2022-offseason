package frc.robot.Meth;

import edu.wpi.first.math.controller.PIDController;

public class MyPIDController extends PIDController{

    private double kff;
    public MyPIDController(double kp, double ki, double kd, double kff) {
        super(kp, ki, kd);
        //TODO Auto-generated constructor stub
        this.kff = kff;
    }
    @Override
    public double calculate(double messurment)
    {
        return super.calculate(messurment) + this.kff * this.getSetpoint();
    }
    public void setFF(double kff)
    {
        this.kff = kff;
    }
    public double getFF()
    {
        return this.kff;
    }
    
}
