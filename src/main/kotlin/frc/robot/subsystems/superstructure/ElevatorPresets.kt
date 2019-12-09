package frc.robot.subsystems.superstructure
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import frc.robot.subsystems.superstructure.ElevatorPresets
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit

object ElevatorPresets {
    class elevatorGoToPreset(val height : SIUnit <Meter>) : FalconCommand(Elevator) {
        //TODO Change to real presets
        override fun initialize() {
            super.initialize()
            Elevator.master.setPosition(height)
        }
        override fun isFinished(): Boolean {
            return (Elevator.master.encoder.position -height).absoluteValue < 0.5.inches
        }
    }
}