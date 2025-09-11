from collections import deque
import serial
import time
import sys
# trajectory generator
# θ

initial_pos_joint1=90
initial_pos_joint2=90
initial_pos_joint3=90
end_eff_open=90
end_eff_close=60

delivery_point_joint1=150
delivery_point_joint2=60
delivery_point_joint3=50


def cubic_polynomial_trajectory_generator(θi,θf,t,n):
    a0=θi
    a1=0
    a2=3*(θf-θi)/(t**2)
    a3=-2*(θf-θi)/(t**3)
    i=1
    point_list=[]
    while(i<=20):
        step=i*t/n
        step_inc=a0+a1*step+a2*step**2+a3*step**3
        step_inc_round=round(step_inc,3)
        point_list.append(step_inc_round)
        i=i+1
    return point_list

def end_eff_operation(θi,θf,n):
    actuation=abs(θi-θf)
    increment=actuation/n
    actuation_points=[]
    if θi<θf:
        step=θi
        i=1
        while(i<=n):
            step=step+increment
            step_round=round(step,3)
            actuation_points.append(step_round)
            i=i+1

    elif θi>θf:
        step=θi
        i=1
        while(i<=n):
            step=step-increment
            step_round=round(step,3)
            actuation_points.append(step_round)
            i=i+1

    return actuation_points        


arduino_data=serial.Serial('/dev/ttyACM0',9600)
time.sleep(2)




def serial_transmit(angle):
    arduino_data.write(f"{angle}\n".encode())


# operation 1= reaching to the desired co ordinates  
joint1_path_op1=cubic_polynomial_trajectory_generator(initial_pos_joint1,29,10,20)
joint2_path_op1=cubic_polynomial_trajectory_generator(initial_pos_joint2,45,10,20)
joint3_path_op1=cubic_polynomial_trajectory_generator(initial_pos_joint3,120,10,20)
end_eff_opening=end_eff_operation(end_eff_close,end_eff_open,20)
trajectory_path_op1=list(zip(joint1_path_op1,joint2_path_op1,joint3_path_op1,end_eff_opening))

# operation 2= closing the end effector
end_eff_closing_op2=end_eff_operation(end_eff_open,end_eff_close,20)

# operation 3= reaching to the dump station
joint1_path_op3=cubic_polynomial_trajectory_generator(29,delivery_point_joint1,10,20)
joint2_path_op3=cubic_polynomial_trajectory_generator(45,delivery_point_joint2,10,20)
joint3_path_op3=cubic_polynomial_trajectory_generator(120,delivery_point_joint3,10,20)
trajectory_path_op3=list(zip(joint1_path_op3,joint2_path_op3,joint3_path_op3))

# operation 4= droping the object
end_eff_opening_op4=end_eff_operation(end_eff_close,end_eff_open,10)

# operation 5= returning to the initial position
joint1_path_op5=cubic_polynomial_trajectory_generator(delivery_point_joint1,initial_pos_joint1,10,20)
joint2_path_op5=cubic_polynomial_trajectory_generator(delivery_point_joint2,initial_pos_joint2,10,20)
joint3_path_op5=cubic_polynomial_trajectory_generator(delivery_point_joint3,initial_pos_joint3,10,20)
end_eff_closing_op5=end_eff_operation(end_eff_open,end_eff_close,20)
trajectory_path_op5=list(zip(joint1_path_op5,joint2_path_op5,joint3_path_op5,end_eff_closing_op5))


# print(joint1_path_op1)
# print(joint2_path_op1)
# print(joint3_path_op1)
# print(trajectory_path_op1)
# latest_response=None
# print(end_eff_opening)
# print(end_eff_closing_op2)


def trajectory_executor(trajectory_path):
    for angle_set in trajectory_path:
        while True:
            serial_transmit(",".join(map(str,angle_set)))
            print(angle_set)
            time.sleep(0.1)
        
            response=arduino_data.readline().decode().strip()
            if response== "ACK":
                print('Arduino Acknowleged')
                break
            elif response=="":
                print('No acknowledgement recieved')
                sys.exit(1)


def end_eff_execution(trajectory):
    for angle in trajectory:
        while True:
            serial_transmit(angle)
            print(f"Sending: {angle}")
            time.sleep(0.1)

            response=arduino_data.readline().decode().strip()
            if response=="ACK":
                print('arduino acknowledged')
                # latest_response=response
                break
            elif response=="":
                print('No acknowledgement recieved')
                sys.exit(1)



trajectory_executor(trajectory_path_op1)
end_eff_execution(end_eff_closing_op2)
trajectory_executor(trajectory_path_op3)
end_eff_execution(end_eff_opening_op4)
trajectory_executor(trajectory_path_op5)

arduino_data.close()
# print(latest_response)
