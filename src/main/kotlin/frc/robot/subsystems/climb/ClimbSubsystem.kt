package frc.robot.subsystems.climb

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.SpeedController
//import edu.wpi.first.wpilibj2.command.Command
//import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.subsystems.superstructure.Proximal
import frc.robot.subsystems.superstructure.Superstructure
import frc.robot.subsystems.superstructure.Wrist
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX

object ClimbSubsystem : FalconSubsystem(){
    val zero = 25.inches
    //Corrects for the ration for where the encoder is
    val habClimberStilt: FalconMAX <Meter> = FalconMAX(9, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitLengthModel(1.nativeUnits * 42.0, 0.75.inches)).apply {
        setPIDGains(1.0 * 7.0 / 3.0, 0.0)
        brakeMode = false
        canSparkMax.setSmartCurrentLimit(40, 40)
        encoder.resetPosition(zero) //setting to zero
        encoder.canEncoder.positionConversionFactor = -1.0 //inverting
    }
    class StiltSend(val stiltHeight : SIUnit <Meter>) : FalconCommand(ClimbSubsystem) {
        override fun initialize() {
            ClimbSubsystem.habClimberStilt.setPosition(stiltHeight)
        }
        override fun isFinished(): Boolean {
            return (ClimbSubsystem.habClimberStilt.encoder.position -stiltHeight).absoluteValue < 0.5.inches
        }


    }
    val intakeWheels = FalconSRX(45, DefaultNativeUnitModel)
    //fun intakeHab(val intakeWheels = FalconSRX(45, DefaultNativeUnitModel)){

   // }
}
//setting pid info (not tuning)
fun <K : SIKey> FalconMAX<K>.setPIDGains(p: Double, d: Double) {
    controller.p = p
    controller.d = d
}

