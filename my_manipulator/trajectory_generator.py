from collections import deque
import serial
import time
import sys
# trajectory generator
# θ

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

arduino_data=serial.Serial('/dev/ttyACM0',9600)
time.sleep(2)




def serial_transmit(angle):
    arduino_data.write(f"{angle}\n".encode())
   
joint1_path=cubic_polynomial_trajectory_generator(90,29,10,20)
joint2_path=cubic_polynomial_trajectory_generator(90,45,10,20)
joint3_path=cubic_polynomial_trajectory_generator(100,120,10,20)

trajectory_path_op1=list(zip(joint1_path,joint2_path,joint3_path))


# print(joint1_path)
# print(joint2_path)
# print(joint3_path)
# print(trajectory_path_op1)
# latest_response=None


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


end_eff_execution(joint1_path)

arduino_data.close()
# print(latest_response)
