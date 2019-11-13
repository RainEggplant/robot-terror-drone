# %%
import numpy as np


actions={}
actions['left']=1
actions['right']=2
actions['forward']=3

#plan to act
def plan2act(plan_act):
    for i in plan_act:
        print(i)

        
# %%
W=320
H=240
H_THRESHOLD=200
FALL_THRESHOLD=100
RUNNING=0
rightflag=1

# pos = {}
# pos['landmine'] = [(1, 150), (2, 140)]
# pos['brink'] = []
# pos['light'] = ['green', 'yellow']

class ActionPlanning(object):
    def __init__(self, posdata):
        self.posdata = posdata

    def plan_action(self):
        while True:
            #get pos
            pos = self.posdata
            #replace the above with the getpos function
            
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
                #brink judge
                for i in pos['brink']:
                    if FALL_THRESHOLD<i[0]<W/2:
                        rightflag=1
                        print('find the left line')
                    if W/2<i[0]<W-FALL_THRESHOLD:
                        rightflag=0
                        print('find the right line')
                #landmine judge
                maxh=max(pos['landmine'][1,:])
                if  maxh<H_THRESHOLD:
                    plan_act.append(actions['forward'])
                elif rightflag:
                    plan_act.append(actions['right'])
                else:
                    plan_act.append(actions['left'])
                plan2act(plan_act)
            break
            sleep(0.01)