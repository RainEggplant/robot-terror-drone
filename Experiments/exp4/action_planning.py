# %%
import numpy as np
import time
import Serial_Servo_Running as SSR



#plan to act
def plan2act(plan_act):
    for i in plan_act:
        SSR.running_action_group(i, 1)
        print(i)

        
# %%
W=320
H=240
H_THRESHOLD=100
FALL_THRESHOLD=20
RUNNING=0


# pos = {}
# pos['landmine'] = [(1, 150), (2, 140)]
# pos['brink'] = []
# pos['light'] = ['green', 'yellow']

class ActionPlanning(object):
    def __init__(self, posdata):
        self.posdata = posdata

    def plan_action(self,rightflag):
        
        #get pos
        pos = self.posdata
        #replace the above with the getpos function
        global RUNNING
        #judge the color
        for i in pos['light']:
            if i=='green':
                RUNNING=1
                print('green light')
                continue
            if i=='red':
                RUNNING=0
                print('red light')
                continue
        #action planning and action excution
        if RUNNING:
            plan_act=[]
            line_judge=0
            #brink judge
            for i in pos['brink']:
                if FALL_THRESHOLD<i[0]<W/2:
                    rightflag=1
                    print('find the left line')
                    time.sleep(1)
                    plan_act.append('custom/move_right')
                    line_judge=1;
                if W/2<i[0]<W-FALL_THRESHOLD:
                    rightflag=0
                    print('find the right line')
                    time.sleep(1)
                    plan_act.append('custom/move_left')
                    line_judge=1;
            #landmine judge
            if (len(pos['landmine'])>0):
                maxh=max(np.array(pos['landmine']).reshape(2,-1)[1,:])
                if  maxh<H_THRESHOLD:
                    plan_act.append('custom/walk')
                elif rightflag:
                    print('find the landmine')
                    time.sleep(1)
                    plan_act.append('custom/move_right')
                else:
                    print('find the landmine')
                    time.sleep(1)
                    plan_act.append('custom/move_left')
            elif line_judge==0:
                plan_act.append('custom/walk')
            plan2act(plan_act)
        time.sleep(0.01)
        return rightflag
