"""
This files odrive model with the Viam Registry.
"""

from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.components.movement_sensor.movement_sensor import MovementSensor
from .my_odometry import MyOdometry

Registry.register_resource_creator(MovementSensor.SUBTYPE, MyOdometry.MODEL, ResourceCreatorRegistration(MyOdometry.new, MyOdometry.validate_config))