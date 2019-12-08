package frc.robot.subsystems.superstructure

import org.ghrobotics.lib.commands.FalconSubsystem
import com.ctre.phoenix.CANifier
import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice
import com.ctre.phoenix.motorcontrol.RemoteSensorSource
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Constants
import frc.robot.Ports
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.nativeunit.toNativeUnitPosition
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.team5940.pantry.lib.ConcurrentFalconJoint
import org.team5940.pantry.lib.MultiMotorTransmission
import org.team5940.pantry.lib.asPWMSource

object Wrist : FalconSubsystem()
 //hello there

