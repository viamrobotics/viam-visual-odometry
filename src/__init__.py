"""
This files odrive model with the Viam Registry.
"""

from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.components.movement_sensor.movement_sensor import MovementSensor
from .odometry_module import Odometry

Registry.register_resource_creator(MovementSensor.SUBTYPE, Odometry.MODEL, ResourceCreatorRegistration(Odometry.new, Odometry.validate_config))