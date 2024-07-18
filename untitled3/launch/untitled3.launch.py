from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace= "" , package= "untitled3" ,
                executable= "Robot_Server" , output = "screen"),
            Node(
                namespace= "" , package= "untitled3" ,
                executable= "Robot_Control" , output = "screen"),
            Node(
                namespace= "" , package= "untitled3" ,
                executable= "Guest_detect" , output = "screen"),
            Node(
                namespace= "" , package= "untitled3" ,
                executable= "Cup_detect" , output = "screen"),
            Node(
                namespace= "" , package= "untitled3" ,
                executable= "UI" , output = "screen"),
            Node(
                namespace= "" , package= "untitled3" ,
                executable= "Voice_Input" , output = "screen"),
        ]
    )
