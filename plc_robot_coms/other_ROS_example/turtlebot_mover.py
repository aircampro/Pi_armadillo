# Turtlebot robot movement (x,y) plane from speach
# modified for xy plane and speach recog by ACP
# modified from https://github.com/prathamv0811/LLM-Navigation_gemini/tree/main
#
import os
import rclpy
import json
import time
import threading
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import google.generativeai as genai
import pyttsx3
from google.generativeai.types import FunctionDeclaration, Tool
import speach_recog_lib

class LLMAgentNode(Node):
    """
    A ROS 2 node that acts as an agent, using an LLM to translate
    natural language commands into robot actions.
    """
    def __init__(self):
        super().__init__('llm_agent_node')
        self.get_logger().info("LLM Agent Node started.")

        # Ensure the Google AI API key is set
        if 'GOOGLE_API_KEY' not in os.environ:
            msg = "GOOGLE_API_KEY environment variable not set! Cannot start node."
            self.get_logger().fatal(msg)
            raise RuntimeError(msg)

        # Configure the generative AI client
        genai.configure(api_key=os.environ['GOOGLE_API_KEY'])
        
        # --- ROS 2 Communication Setup ---
        # Odometry subscriber to get robot's current position
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.current_pose = None
        self.odom_received_event = threading.Event()
        self.get_logger().info("Subscribed to /odom topic.")

        # Create a publisher for the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Publisher for /cmd_vel created.")

        # Define the "tools" (functions) the LLM can call
        self.available_tools = {
            "move_robot": self.move_robot,
            "rotate_robot": self.rotate_robot,
        }

        # --- LLM and Tool Setup ---
        tools_for_google = Tool(
            function_declarations=[
                FunctionDeclaration(
                    name="move_robot",
                    description="Moves the robot forward or backward or left or right by a specified distance.",
                    parameters={
                        "type": "object",
                        "properties": {
                            "distance": {"type": "number", "description": "The distance to move in meters."},
                            "direction": {"type": "string", "enum": ["forward", "backward", "left", "right", "forwardleft", "forwardright", "backwardleft", "backwardright", "leftforward", "rightforward", "leftbackward", "rightbackward" ]},
                        },
                        "required": ["distance", "direction"],
                    },
                ),
                FunctionDeclaration(
                    name="rotate_robot",
                    description="Rotates the robot clockwise or counter-clockwise by a specified angle.",
                    parameters={
                        "type": "object",
                        "properties": {
                            "angle": {"type": "number", "description": "The angle to rotate in degrees."},
                            "direction": {"type": "string", "enum": ["clockwise", "counter-clockwise"]},
                        },
                        "required": ["angle", "direction"],
                    },
                ),
            ]
        )

        # Initialize the Generative Model with the tools
        self.model = genai.GenerativeModel(
            model_name='gemini-1.5-flash-latest',                                                                      # A fast and capable model for tool use
            tools=[tools_for_google]
        )
        self.get_logger().info("Google Gemini model initialized.")

        # Initialize the TTS engine
        try:
            self.tts_engine = pyttsx3.init()
            self.get_logger().info("TTS engine initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TTS engine: {e}")
            self.tts_engine = None

    def odom_callback(self, msg: Odometry):
        """Callback to continuously update the robot's current pose."""
        self.current_pose = msg.pose.pose
        if not self.odom_received_event.is_set():
            self.get_logger().info("First odometry message received.")
            self.odom_received_event.set()

    def get_yaw_from_quaternion(self, quaternion):
        """Convert quaternion to Euler angles and return yaw."""
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        # Euler angle conversion
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

    def speak(self, text):
        """Uses the TTS engine to speak the given text."""
        if self.tts_engine:
            self.get_logger().info(f"Speaking: '{text}'")
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()

    def move_robot(self, distance: float, direction: str):
        """Generates and publishes a Twist message to move the robot."""
        self.get_logger().info(f"Executing move_robot: distance={distance}, direction='{direction}'")
        if self.current_pose is None:
            self.get_logger().error("Robot pose not available yet. Cannot execute move.")
            self.speak("I don't know my current position, so I cannot move.")
            return "Error: Robot pose not available."

        self.speak(f"Moving {direction} by {distance} meters.")
        
        start_pose = self.current_pose
        distance_traveled = 0.0
        rate = self.create_rate(10)                                                         # ROS Rate 10 Hz

        twist_msg = Twist()
        speed = 0.2                                                                         # default speed in m/s
        if not direction.find("right") == -1:
            twist_msg.linear.y = speed  
        elif not direction.find("left") == -1:
            twist_msg.linear.y = -speed 
        else:
            twist_msg.linear.y = 0
        if not direction.find("forward") == -1:
            twist_msg.linear.x = speed  
        elif not direction.find("backward") == -1:
            twist_msg.linear.x = -speed 
        else:
            twist_msg.linear.x = 0		   

        while distance_traveled < distance:
            if self.current_pose is None:
                self.get_logger().warn("Pose data lost during movement.")
                break
            self.publisher_.publish(twist_msg)
            dx = self.current_pose.position.x - start_pose.position.x
            dy = self.current_pose.position.y - start_pose.position.y
            distance_traveled = math.sqrt(dx*dx + dy*dy)
            rate.sleep()
        
        # Stop the robot
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info("Movement complete.")
        return f"Successfully moved {direction} by {distance} meters."

    def rotate_robot(self, angle: float, direction: str):
        """Generates and publishes a Twist message to rotate the robot."""
        self.get_logger().info(f"Executing rotate_robot: angle={angle}, direction='{direction}'")
        if self.current_pose is None:
            self.get_logger().error("Robot pose not available yet. Cannot execute rotate.")
            self.speak("I don't know my current orientation, so I cannot rotate.")
            return "Error: Robot pose not available."

        self.speak(f"Rotating {direction} by {angle} degrees.")

        start_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        target_rad = math.radians(angle)
        angle_rotated = 0.0
        rate = self.create_rate(10)                                                                                                      # 10 Hz

        twist_msg = Twist()
        angular_speed = 0.5  # rad/s
        twist_msg.angular.z = -angular_speed if direction == "clockwise" else angular_speed

        while angle_rotated < target_rad:
            if self.current_pose is None:
                self.get_logger().warn("Pose data lost during rotation.")
                break
            self.publisher_.publish(twist_msg)
            current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
            # Calculate delta, handling angle wrap-around
            delta_yaw = abs(current_yaw - start_yaw)
            if delta_yaw > math.pi:
                delta_yaw = 2 * math.pi - delta_yaw
            angle_rotated = delta_yaw
            rate.sleep()

        # Stop the robot
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().info("Rotation complete.")
        return f"Successfully rotated {direction} by {angle} degrees."

    def run_agent_loop(self):
        """Main loop to get user input from the terminal and interact with the LLM."""
        self.get_logger().info("Waiting for the first odometry message to initialize...")

        # Wait for the odom_received_event to be set, with a timeout.
        # This ensures we don't start accepting commands before the robot knows its position.
        if not self.odom_received_event.wait(timeout=10.0):
            self.get_logger().error("Timed out waiting for odometry data. The /odom topic might not be publishing.")
            self.speak("I can't determine my location. Please check the robot's sensors.")
            return

        self.get_logger().info("Initialization complete. Ready for commands.")
        speach_recog_lib.set_max_rec_vol()                                                             # set recording volume to max
        self.speak("I am now ready for your commands.")

        while rclpy.ok():
            try:
                # command = input("Enter a command for the robot (or 'quit'): ")
				command = speach_recog_lib.translate_speach()
                command = command.lower()
                if command == 'quit':
                    break

                self.get_logger().info(f"Sending command to LLM: '{command}'")
                
                # Send the command to the Gemini model
                response = self.model.generate_content(command)
                response_part = response.candidates[0].content.parts[0]

                # Check if the model decided to call a function
                if response_part.function_call:
                    function_call = response_part.function_call
                    function_name = function_call.name
                    function_to_call = self.available_tools.get(function_name)

                    if function_to_call:
                        # The arguments are a dict-like object, no json.loads needed
                        function_args = dict(function_call.args)
                        self.get_logger().info(f"LLM chose function: {function_name} with args: {function_args}")
                        
                        # Execute the function
                        function_response = function_to_call(**function_args)
                        self.get_logger().info(f"Function response: {function_response}")
                    else:
                        self.get_logger().warn(f"LLM chose an unknown function: {function_name}")
                else:
                    llm_response_text = response_part.text
                    self.get_logger().info(f"LLM response: {llm_response_text}")
                    self.speak(llm_response_text)
            
            except Exception as e:
                self.get_logger().error(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)
    llm_agent_node = None
    spin_thread = None
    try:
        llm_agent_node = LLMAgentNode()

        # rclpy.spin() blocks, so we run it in a separate thread.
        # This allows the main thread to handle user input with input().
        spin_thread = threading.Thread(target=rclpy.spin, args=(llm_agent_node,))
        spin_thread.start()

        llm_agent_node.run_agent_loop()

    except (KeyboardInterrupt, RuntimeError) as e:
        if llm_agent_node:
            llm_agent_node.get_logger().info(f"Shutting down: {e}")
        else:
            print(f"Shutting down due to initialization error: {e}")
    finally:
        rclpy.shutdown()
        if spin_thread and spin_thread.is_alive():
            spin_thread.join()
        if llm_agent_node:
            llm_agent_node.destroy_node()

if __name__ == '__main__':
    main()