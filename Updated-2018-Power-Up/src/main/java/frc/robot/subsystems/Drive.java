package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase{

    public WPI_TalonSRX leftTalon1, leftTalon2, rightTalon1, rightTalon2;
    public DifferentialDrive differentialDrive;
    public MotorControllerGroup leftGroup, rightGroup;


    public Drive() {
        this.leftTalon1 = new WPI_TalonSRX(DriveConstants.kleftTalon1);
        this.leftTalon2 = new WPI_TalonSRX(DriveConstants.kleftTalon2);
        this.rightTalon1 = new WPI_TalonSRX(DriveConstants.krightTalon1);
        this.rightTalon2 = new WPI_TalonSRX(DriveConstants.krightTalon2);

        this.leftGroup = new MotorControllerGroup(leftTalon1, leftTalon2);
        this.rightGroup = new MotorControllerGroup(rightTalon1, rightTalon2);

        this.leftGroup.setInverted(true);
        this.rightGroup.setInverted(false);

        this.differentialDrive = new DifferentialDrive(leftGroup, rightGroup);  
    }

    public void tankDrive(double leftInput, double rightInput) {
        // Invert left side so that the robot moves in the correct direction
        this.differentialDrive.tankDrive(leftInput, rightInput);
    }
    
      public void arcadeDrive(double speed, double rotation) {
        this.differentialDrive.arcadeDrive(speed, rotation);
    }
}