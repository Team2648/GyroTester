
package com.zephyr.robots;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	private VictorSP leftMotor;
	private VictorSP rightMotor;
	
	private RobotDrive dt;
	
	private Joystick j1;
	
	private AnalogGyro gyro;
	
	private PIDController controller;
	
	private PIDOutput output;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	leftMotor = new VictorSP(0);
    	rightMotor = new VictorSP(1);
    	
    	dt = new RobotDrive(leftMotor, rightMotor);
    	
    	j1 = new Joystick(0);
    	
    	gyro = new AnalogGyro(0);
    	gyro.setSensitivity(.007);
    	gyro.calibrate();
    	gyro.reset();
    	
    	output = new PIDOutput() {

			@Override
			public void pidWrite(double output) {
				dt.setLeftRightMotorOutputs(output, -output);
			}
    		
    	};
    	
    	SmartDashboard.putNumber("kP", .01);
    	SmartDashboard.putNumber("kI", .01);
    	SmartDashboard.putNumber("kD", 0);
    	
    	controller = new PIDController(SmartDashboard.getNumber("kP"), SmartDashboard.getNumber("kI"), SmartDashboard.getNumber("kD"), gyro, output);
    	controller.setAbsoluteTolerance(.25);
    	controller.disable();
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    }

    
    public void teleopPeriodic() {
    	if(!controller.isEnabled())
    	{
    		dt.arcadeDrive(-j1.getY(), -j1.getX());
    		
    		controller.setPID(SmartDashboard.getNumber("kP"), SmartDashboard.getNumber("kI"), SmartDashboard.getNumber("kD"));
    	}
    	
    	if(controller.isEnabled())
    	{
    		if(controller.onTarget())
    		{
    			controller.disable();
    			dt.setLeftRightMotorOutputs(0, 0);
    		}
    	}
    	
    	if(j1.getRawButton(1))
    	{
    		gyro.reset();
    	}
    	else if(j1.getRawButton(2))
    	{
    		controller.setSetpoint(90);
    		controller.enable();
    	}
    	else if(j1.getRawButton(3))
    	{
    		controller.disable();
    	}
    	
    	
    	SmartDashboard.putNumber("Gyro", gyro.getAngle());
    	SmartDashboard.putBoolean("Controller Enabled", controller.isEnabled());
        
    }
    
    public void disabledPeriodic()
    {
    	SmartDashboard.putNumber("Gyro", gyro.getAngle());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
