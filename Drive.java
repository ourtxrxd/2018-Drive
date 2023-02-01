package org.usfirst.frc.team2445.robot.subsystems;

import org.usfirst.frc.team2445.robot.RobotMap;
import org.usfirst.frc.team2445.robot.commands.drive.CurvatureDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive extends Subsystem {
	public WPI_TalonSRX talonFR, talonBR, talonFL, talonBL;
	public SpeedControllerGroup leftSide, rightSide;
	public ADXRS450_Gyro gyro;
		
	public DifferentialDrive differentialDrive;
	
	private Solenoid shifter;
	private Gear currentGear;
	
	public Drive() {
		
		talonFR = new WPI_TalonSRX(RobotMap.DRIVE_FR_PORT);
		talonBR = new WPI_TalonSRX(RobotMap.DRIVE_BR_PORT);
		talonFL = new WPI_TalonSRX(RobotMap.DRIVE_FL_PORT);
		talonBL = new WPI_TalonSRX(RobotMap.DRIVE_BL_PORT);
	
		rightSide = new SpeedControllerGroup(talonFR, talonBR);
		leftSide = new SpeedControllerGroup(talonFL, talonBL);
		leftSide.setInverted(true);
		differentialDrive = new DifferentialDrive(leftSide, rightSide);
		differentialDrive.setSafetyEnabled(false);
			
		talonFR.setInverted(true);
		talonBR.setInverted(true);
		talonFL.setInverted(false);
		talonBL.setInverted(false);
		
		talonFR.config_kP(RobotMap.FPID_SLOT_ID, RobotMap.TALON_FR_PID_kP, RobotMap.FPID_DELAY);
		talonFR.config_kI(RobotMap.FPID_SLOT_ID, RobotMap.TALON_FR_PID_kI, RobotMap.FPID_DELAY);
		talonFR.config_kD(RobotMap.FPID_SLOT_ID, RobotMap.TALON_FR_PID_kD, RobotMap.FPID_DELAY);
		talonFR.config_kF(RobotMap.FPID_SLOT_ID, RobotMap.TALON_FR_PID_kF, RobotMap.FPID_DELAY);
		
		talonFL.config_kP(RobotMap.FPID_SLOT_ID, RobotMap.TALON_FL_PID_kP, RobotMap.FPID_DELAY);
		talonFL.config_kI(RobotMap.FPID_SLOT_ID, RobotMap.TALON_FL_PID_kI, RobotMap.FPID_DELAY);
		talonFL.config_kD(RobotMap.FPID_SLOT_ID, RobotMap.TALON_FL_PID_kD, RobotMap.FPID_DELAY);
		talonFL.config_kF(RobotMap.FPID_SLOT_ID, RobotMap.TALON_FL_PID_kF, RobotMap.FPID_DELAY);
		
		talonFR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		talonFL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		
		talonFR.setSensorPhase(false);
		talonFL.setSensorPhase(false);
		
		
		gyro = new ADXRS450_Gyro();
		shifter = new Solenoid(RobotMap.SHIFTER_SOLENOID_PORT);
		currentGear = Gear.LOW;
		
		System.out.println("Drive Initialized");
		
		this.resetEncoders();
	}
	
	public void shift(Gear gear) {
		this.shifter.set(gear.get());
		currentGear = gear;
	}
	
	public enum Gear {
		HIGH(false),
		LOW(true);
		
		private boolean value;
		
		Gear(boolean value) {
			this.value = value;
		}
		
		public boolean get() {
			return this.value;
		}
	}
	
	public Gear getCurrentGear() {
		return this.currentGear;
	}
	public Boolean getCurrentGearBoolean() {
		return this.currentGear.get();
	}
	
	public enum Direction {
		FORWARD,
		BACKWARD
	}
	
	/**
	 * Used to move the robot in one singluar direction based on percent output1
	 * 
	 * @param rightY (Double) Right Y input
	 * @param leftY (Double) Left Y input
	 */
	public void tankPercentDrive(double leftY, double rightY) {
		talonFR.set(ControlMode.PercentOutput, rightY);
		talonBR.set(ControlMode.Follower, RobotMap.DRIVE_FR_PORT);
		talonFL.set(ControlMode.PercentOutput, leftY);
		talonBL.set(ControlMode.Follower, RobotMap.DRIVE_FL_PORT);
	}
	public void tankVelocityDrive(double leftVelocity, double rightVelocity) {
		talonFR.set(ControlMode.Velocity, rightVelocity); 
		talonBR.set(ControlMode.Follower, RobotMap.DRIVE_FR_PORT);
		talonFL.set(ControlMode.Velocity, leftVelocity);
		talonBL.set(ControlMode.Follower, RobotMap.DRIVE_FL_PORT);
	}
	public void tankVelocityDriveRight(double rightVelocity) {
		talonFR.set(ControlMode.Velocity, rightVelocity); 
		talonBR.set(ControlMode.Follower, RobotMap.DRIVE_FR_PORT);
	}
	public void tankVelocityDriveLeft(double leftVelocity) {
		talonFL.set(ControlMode.Velocity, leftVelocity);
		talonBL.set(ControlMode.Follower, RobotMap.DRIVE_FL_PORT);
	}
	
	public double limit(double value) {
	    if (value > 1.0) {
	      return 1.0;
	    }
	    if (value < -1.0) {
	      return -1.0;
	    }
	    return value;
	}
	
	 public double velocityToPercent(double xSpeed) {
		return xSpeed/RobotMap.MAX_DRIVE_VELOCITY;
	}
	
	public double percentToVelocity(double xSpeedVelocity) {
		return xSpeedVelocity*RobotMap.MAX_DRIVE_VELOCITY;
	} 
	
	public void differentialArcadeDrive(double xSpeedVelocity, double zRotation, boolean squaredInputs) {
	    double leftMotorOutput;
	    double rightMotorOutput;
	    double xSpeed;
	    
	    xSpeed = velocityToPercent(xSpeedVelocity);
	    
	    zRotation = limit(zRotation);
	    xSpeed = limit(xSpeed);
	    if (squaredInputs) {
	        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
	        zRotation = Math.copySign(zRotation * zRotation, zRotation);
	      }
	    
		double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
		
		if (xSpeed >= 0.0) {
		      // First quadrant, else second quadrant
		      if (zRotation >= 0.0) {
		        leftMotorOutput = maxInput;
		        rightMotorOutput = xSpeed - zRotation;
		      } else {
		        leftMotorOutput = xSpeed + zRotation;
		        rightMotorOutput = maxInput;
		      }
		    } else {
		      // Third quadrant, else fourth quadrant
		      if (zRotation >= 0.0) {
		        leftMotorOutput = xSpeed + zRotation;
		        rightMotorOutput = maxInput;
		      } else {
		        leftMotorOutput = maxInput;
		        rightMotorOutput = xSpeed - zRotation;
		      }
		    }
		//
		rightMotorOutput = limit(rightMotorOutput);
		leftMotorOutput = limit(leftMotorOutput);
		
		rightMotorOutput = percentToVelocity(rightMotorOutput);
		leftMotorOutput = percentToVelocity(leftMotorOutput);

		talonFL.set(ControlMode.Velocity,leftMotorOutput);
		talonFR.set(ControlMode.Velocity,rightMotorOutput);
	}
//	/**
//	 * Used to move the robot in one singluar direction based on position
//	 * 
//	 * @param velocity (Int) Distance in inches
//	 */
//	public void velocityDrive(int velocity) {
//		this.talonFR.set(ControlMode.Velocity, velocity);
//		this.talonBR.set(ControlMode.Follower, RobotMap.TALON_FR_PORT);
//		this.talonFL.set(ControlMode.Velocity, velocity);
//		this.talonBL.set(ControlMode.Follower, RobotMap.TALON_FL_PORT);
//		System.out.println("FR: " + talonFR.getSensorCollection().getQuadraturePosition());
//		System.out.println("FL: " + talonFL.getSensorCollection().getQuadraturePosition());
//	}
//	
//	/**
//	 * Used to change the angle of the robot relative to the current direction of the robot
//	 * CW = negative Angle
//	 * CCW = positive Angle
//	 * 
//	 * @param angle (Double) Angle which is set (takes degrees)
//	 */
//	public void turnDrive(int angle, int velocity) {
//		this.talonFR.set(ControlMode.Position, -velocity);
//		this.talonBR.set(ControlMode.Position, -ticks);
//		this.talonFL.set(ControlMode.Position, velocity);
//		this.talonBL.set(ControlMode.Position, ticks);
//	}
//	public void turnDrive(double angle) {
//		double inches = 0;
//		double ticks = 0;
//		double radian = 0;
//		radian = Math.toRadians(angle);
//		inches = radian*(RobotMap.MIDDLE_TO_WHEEL/2);
//		ticks = distanceToTicks(inches);
//		this.talonFR.set(ControlMode.Position, -ticks);
//		this.talonBR.set(ControlMode.Position, -ticks);
//		this.talonFL.set(ControlMode.Position, ticks);
//		this.talonBL.set(ControlMode.Position, ticks);
//	}
	/**
	 * Resets all four encoders for the drive
	 */
	public void resetEncoders() {
		this.setEncoderPosition(Encoder.LEFT, 0);
		this.setEncoderPosition(Encoder.RIGHT, 0);
	}
	
	public enum Encoder {
		LEFT,
		RIGHT;
	}
	
	public int getEncoderPosition(Encoder encoder) {
		switch(encoder) {
			case LEFT:
				return talonFL.getSelectedSensorPosition(0);
			case RIGHT:
				return talonFR.getSelectedSensorPosition(0);
			default:
				return 0;
		}
	}
	
	public int getEncoderVelocity(Encoder encoder) {
		switch(encoder) {
			case LEFT:
				return talonFL.getSelectedSensorVelocity(0);
			case RIGHT:
				return talonFR.getSelectedSensorVelocity(0);
			default:
				return 2445;
		}
	}
	
	public void setEncoderPosition(Encoder encoder, int position) {
		switch(encoder) {
			case LEFT:
				talonFR.setSelectedSensorPosition(position, 0, 10);
				break;
			case RIGHT:
				talonFL.setSelectedSensorPosition(position, 0, 10);
				break;
			default:
				return;
		}
	}
	
    public int distanceToTicks(double inches){
    	return  1 + ((int)( (RobotMap.TICKS_PER_ROTATION_DRIVE * inches) / ( RobotMap.WHEEL_DIAMETER * Math.PI ) ));
    }
    
	public double averageTicks() {
		return (Math.abs(this.getEncoderPosition(Encoder.LEFT)) + Math.abs(this.getEncoderPosition(Encoder.RIGHT))) / 2;
	}

	public double getAverageVelocity() {
		return (Math.abs(this.getEncoderVelocity(Encoder.LEFT)) + Math.abs(this.getEncoderVelocity(Encoder.RIGHT))) / 2;
	}
	
	public void resetGyro() {
		gyro.reset();
	}
	
	public double getGyro() {
		return gyro.getAngle();
	}
	
	public double getVoltage() {
		return talonFR.getMotorOutputVoltage();
	}
	
    public void initDefaultCommand() {
    	this.setDefaultCommand(new CurvatureDrive());
    }
}
