# trajectory generator
# θ

def trajectory_funcntion(θi,θf,t,n):
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



x=trajectory_funcntion(90,60,10,20)

print(x)
