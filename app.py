import asyncio

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.arm import Arm
from viam.components.board import Board
from viam.services.motion import MotionClient
from viam.services.vision import VisionClient

async def connect():
    creds = Credentials(
        type='robot-location-secret',
		# Replace "<SECRET>" (including brackets) with your robot's secret
        payload='<SECRET>')
    opts = RobotClient.Options(
        refresh_interval=0,
        dial_options=DialOptions(credentials=creds)
    )
    return await RobotClient.at_address('mechanodroid-main.168teukasm.viam.cloud', opts)

async def main():
    robot = await connect()

    print('Resources:')
    print(robot.resource_names)
    
    # MecaTech
    meca_tech = Arm.from_robot(robot, "MecaTech")
    meca_tech_return_value = await meca_tech.get_end_position()
    print(f"MecaTech get_end_position return value: {meca_tech_return_value}")
  
    # Note that the pin supplied is a placeholder. Please change this to a valid pin you are using.
    # MechaBoard
    mecha_board = Board.from_robot(robot, "MechaBoard")
    mecha_board_return_value = await mecha_board.gpio_pin_by_name("16")
    print(f"MechaBoard gpio_pin_by_name return value: {mecha_board_return_value}")
  
    """
    # Note this function requires additional arguments.
    # Please provide valid arguments for function to work properly 
    # Dynamic-Obstacle-Avoidance
    dynamic_obstacle_avoidance = MotionClient.from_robot(robot, "Dynamic-Obstacle-Avoidance")
    dynamic_obstacle_avoidance_return_value = await dynamic_obstacle_avoidance.get_pose()
    print(f"Dynamic-Obstacle-Avoidance get_pose return value: {dynamic_obstacle_avoidance_return_value}")
    """
  
    """
    # Note this function requires additional arguments.
    # Please provide valid arguments for function to work properly 
    # Smooth-Path-Planning
    smooth_path_planning = MotionClient.from_robot(robot, "Smooth-Path-Planning")
    smooth_path_planning_return_value = await smooth_path_planning.get_pose()
    print(f"Smooth-Path-Planning get_pose return value: {smooth_path_planning_return_value}")
    """
  
    """
    # Note this function requires additional arguments.
    # Please provide valid arguments for function to work properly 
    # Gesture-Based-Control
    gesture_based_control = MotionClient.from_robot(robot, "Gesture-Based-Control")
    gesture_based_control_return_value = await gesture_based_control.get_pose()
    print(f"Gesture-Based-Control get_pose return value: {gesture_based_control_return_value}")
    """
  
    """
    # Note this function requires additional arguments.
    # Please provide valid arguments for function to work properly 
    # Object-Tracking
    object_tracking = MotionClient.from_robot(robot, "Object-Tracking")
    object_tracking_return_value = await object_tracking.get_pose()
    print(f"Object-Tracking get_pose return value: {object_tracking_return_value}")
    """
  
    """
    # Note this function requires additional arguments.
    # Please provide valid arguments for function to work properly 
    # Voice-Activated-Movement
    voice_activated_movement = MotionClient.from_robot(robot, "Voice-Activated-Movement")
    voice_activated_movement_return_value = await voice_activated_movement.get_pose()
    print(f"Voice-Activated-Movement get_pose return value: {voice_activated_movement_return_value}")
    """
  
    """
    # Note this function requires additional arguments.
    # Please provide valid arguments for function to work properly 
    # Customizable-Movement-Patterns
    customizable_movement_patterns = MotionClient.from_robot(robot, "Customizable-Movement-Patterns")
    customizable_movement_patterns_return_value = await customizable_movement_patterns.get_pose()
    print(f"Customizable-Movement-Patterns get_pose return value: {customizable_movement_patterns_return_value}")
    """
  
    """
    # Note this function requires additional arguments.
    # Please provide valid arguments for function to work properly 
    # Remote-Control
    remote_control = MotionClient.from_robot(robot, "Remote-Control")
    remote_control_return_value = await remote_control.get_pose()
    print(f"Remote-Control get_pose return value: {remote_control_return_value}")
    """
  
    """
    # Note this function requires additional arguments.
    # Please provide valid arguments for function to work properly 
    # Autonomous-Charging
    autonomous_charging = MotionClient.from_robot(robot, "Autonomous-Charging")
    autonomous_charging_return_value = await autonomous_charging.get_pose()
    print(f"Autonomous-Charging get_pose return value: {autonomous_charging_return_value}")
    """
  
    """
    # Note this function requires additional arguments.
    # Please provide valid arguments for function to work properly 
    # Terrain-Adaptability
    terrain_adaptability = MotionClient.from_robot(robot, "Terrain-Adaptability")
    terrain_adaptability_return_value = await terrain_adaptability.get_pose()
    print(f"Terrain-Adaptability get_pose return value: {terrain_adaptability_return_value}")
    """
  
    """
    # Note this function requires additional arguments.
    # Please provide valid arguments for function to work properly 
    # Emergency-Stop
    emergency_stop = MotionClient.from_robot(robot, "Emergency-Stop")
    emergency_stop_return_value = await emergency_stop.get_pose()
    print(f"Emergency-Stop get_pose return value: {emergency_stop_return_value}")
    """
  
    # Note that the Camera supplied is a placeholder. Please change this to a valid Camera.
    # OptiDroid
    opti_droid = VisionClient.from_robot(robot, "OptiDroid")
    opti_droid_return_value = await opti_droid.get_classifications_from_camera(YOURCAMERANAME, 1)
    print(f"OptiDroid get_classifications_from_camera return value: {opti_droid_return_value}")
  
    # Note that the Camera supplied is a placeholder. Please change this to a valid Camera.
    # DroidVision
    droid_vision = VisionClient.from_robot(robot, "DroidVision")
    droid_vision_return_value = await droid_vision.get_classifications_from_camera(YOURCAMERANAME, 1)
    print(f"DroidVision get_classifications_from_camera return value: {droid_vision_return_value}")
  
    # Note that the Camera supplied is a placeholder. Please change this to a valid Camera.
    # VisionBot
    vision_bot = VisionClient.from_robot(robot, "VisionBot")
    vision_bot_return_value = await vision_bot.get_classifications_from_camera(YOURCAMERANAME, 1)
    print(f"VisionBot get_classifications_from_camera return value: {vision_bot_return_value}")
  
    # Note that the Camera supplied is a placeholder. Please change this to a valid Camera.
    # MechanoVision
    mechano_vision = VisionClient.from_robot(robot, "MechanoVision")
    mechano_vision_return_value = await mechano_vision.get_classifications_from_camera(YOURCAMERANAME, 1)
    print(f"MechanoVision get_classifications_from_camera return value: {mechano_vision_return_value}")
  
    # Note that the Camera supplied is a placeholder. Please change this to a valid Camera.
    # VisualBot
    visual_bot = VisionClient.from_robot(robot, "VisualBot")
    visual_bot_return_value = await visual_bot.get_classifications_from_camera(YOURCAMERANAME, 1)
    print(f"VisualBot get_classifications_from_camera return value: {visual_bot_return_value}")

    # Don't forget to close the robot when you're done!
    await robot.close()

if __name__ == '__main__':
    asyncio.run(main())
