import asyncio

from viam.module.module import Module
from .odometry_module import Odometry
from viam.components.movement_sensor.movement_sensor import MovementSensor


async def main():
    """This function creates and starts a new module, after adding all desired resources.
    Resources must be pre-registered. For an example, see the `gizmo.__init__.py` file.
    """

    module = Module.from_args()
    module.add_model_from_registry(MovementSensor.SUBTYPE, Odometry.MODEL)
    await module.start()


if __name__ == "__main__":
    asyncio.run(main())
