package frc.robot.subsystems.climb

import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX

object ClimbSubsystem : FalconSubsystem(){
    val zero = 25.inches
    //Corrects for the ration for where the encoder is
    val habClimberStilt: FalconMAX<Meter> = FalconMAX(9, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitLengthModel(1.nativeUnits * 42.0, 0.75.inches)).apply {
        setPIDGains(1.0 * 7.0 / 3.0, 0.0)
        brakeMode = false
        canSparkMax.setSmartCurrentLimit(40, 40)
        encoder.resetPosition(zero) //setting to zero
        encoder.canEncoder.positionConversionFactor = -1.0 //inverting
    }
}
//setting pid info (not tuning)
fun <K : SIKey> FalconMAX<K>.setPIDGains(p: Double, d: Double) {
    controller.p = p
    controller.d = d
}
