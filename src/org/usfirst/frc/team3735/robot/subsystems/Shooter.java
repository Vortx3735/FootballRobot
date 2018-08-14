/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3735.robot.subsystems;

import org.usfirst.frc.team3735.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Shooter extends Subsystem {
	
	private WPI_TalonSRX l1;
	private WPI_TalonSRX l2;
	private WPI_TalonSRX r1;
	private WPI_TalonSRX r2;
	

	public Shooter() {
		 l1 = new WPI_TalonSRX(RobotMap.leftShooterMotor1);
		 l2 = new WPI_TalonSRX(RobotMap.leftShooterMotor2);
		 r1 = new WPI_TalonSRX(RobotMap.rightShooterMotor1);
		 r2 = new WPI_TalonSRX(RobotMap.rightShooterMotor2);
		 
		 r2.setInverted(true);

		 l2.follow(l1);
		 r2.follow(r1);
		 SmartDashboard.putNumber("Shooter Speed", 0.2);
	}
	
	public void initDefaultCommand() {
		
	}
	
	public void setLeftRight(double speed) {
		l1.set(ControlMode.PercentOutput, speed);
		r1.set(ControlMode.PercentOutput, speed);
	}
	
	public void log() {
		SmartDashboard.putNumber("L1 motor amps", l1.getOutputCurrent());
		SmartDashboard.putNumber("R1 motor amps", r1.getOutputCurrent());
		SmartDashboard.putNumber("L1 motor voltage", l1.getMotorOutputVoltage());
		SmartDashboard.putNumber("R1 motor voltage", r1.getMotorOutputVoltage());
		SmartDashboard.putNumber("L1 resistance", l1.getMotorOutputVoltage()/l1.getOutputCurrent());
		SmartDashboard.putNumber("R1 resistance", r1.getMotorOutputVoltage()/r1.getOutputCurrent());
		
		SmartDashboard.putNumber("L2 motor amps", l2.getOutputCurrent());
		SmartDashboard.putNumber("R2 motor amps", r2.getOutputCurrent());
		SmartDashboard.putNumber("L2 motor voltage", l2.getMotorOutputVoltage());
		SmartDashboard.putNumber("R2 motor voltage", r2.getMotorOutputVoltage());
		SmartDashboard.putNumber("L2 resistance", l2.getMotorOutputVoltage()/l2.getOutputCurrent());
		SmartDashboard.putNumber("R2 resistance", r2.getMotorOutputVoltage()/r2.getOutputCurrent());

	}
}
