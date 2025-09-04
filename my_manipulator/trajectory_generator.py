from collections import deque
import serial
import time
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

latest_response=None
for angle in joint1_path:
    while True:
        serial_transmit(angle)
        print(f"Sending: {angle}")
        time.sleep(0.1)

        response=arduino_data.readline().decode().strip()
        if response=="ACK":
            print('arduino acknowledged')
            latest_response=response
            break

arduino_data.close()
print(latest_response)
