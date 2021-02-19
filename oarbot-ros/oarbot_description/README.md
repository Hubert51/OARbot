## To view the OARbot model in Gazebo
`roslaunch oarbot_description oarbot.launch`


After initialize the OARbot in gazebo, to control the base of the OARbot(you need to include the teleop_twist_keyboard module in your workspace):
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
