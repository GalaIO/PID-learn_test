__author__ = 'yingjie'
"modeling a PID, contain various ways."

import numpy
import pylab

#位置型PID，无死区，简单PID实现。
#参数说明
#__stable 是PID调节到目标状态（平稳状态），该状态有可能随时间发生变化，可以通过PID_stable来设置，平稳状态
#__bias 是偏差即实际反馈值与目标值的偏差，反映了差别程度
#__lbias 上一次的偏差历史，在求离散系统中微分有用
#__accumulate 表示积分量，即从开始到上一次的积分累积量
#__Kp PID的比例系数
#__Ki PID的积分系数
#__Kd PID的微分系数
class PID_loc(object):
    '''位置型PID，无死区，简单PID实现。
    参数说明
    __stable 是PID调节到目标状态（平稳状态），该状态有可能随时间发生变化，可以通过PID_stable来设置，平稳状态
    __bias 是偏差即实际反馈值与目标值的偏差，反映了差别程度
    __lbias 上一次的偏差历史，在求离散系统中微分有用
    __accumulate 表示积分量，即从开始到上一次的积分累积量
    __Kp PID的比例系数，非负
    __Ki PID的积分系数，非负
    __Kd PID的微分系数，非负'''
    __stable = 0.0
    #__actual = 0.0
    __bias = 0.0
    __lbias = 0.0
    __accumulate = 0.0
    __Kp = 0.0
    __Ki = 0.0
    __Kd = 0.0

    def __init__(self,Kp=3.5,Ki=0.5,Kd=20.0):
        '''
        传入默认参数，或者如数响应PID参数'''
        self.__Kp = Kp
        self.__Ki = Ki
        self.__Kd = Kd

    def PID_pop(self,actual=200.0):
        '''弹出一个PID的控制量，因为PID是一个控制函数，把观测量的偏差输入，
        得到了控制量大小，然后输出控制量，接着在观测变化量反馈，再继续控制'''
        self.__bias = actual - self.__stable
        self.__accumulate += self.__bias
        control = self.__Kp*self.__bias + self.__Ki*self.__accumulate + self.__Kd*(self.__bias-self.__lbias)
        self.__lbias = self.__bias
        return control

    def PID_stable(self,stable=0.0):
        '''设置理想平稳状态'''
        self.__stable = stable

def PID_loc_test(x,l1):
    # 参数调试：对于参数调试来说，最关键的就是比例系数Kp，设置合适的比例系数可以让观测值快速的接近理想状态，但是
    # 设置更大的Kp也意味着抖动也会更加厉害。当反馈量在经过几个周期，距离理想状态还比较远时，需要加大Kp
    # 积分系数Ki，积分系数Ki的话，他会把历史的偏差进行累积，也就是说只要还存在偏差即反馈值-平稳状态>0的话
    # PID的积分量就一直在累积，所以积分系数越大会加快接近理想状态时间，但更大的Ki会引起震荡，同时会更快消除偏差。
    # 当反馈量在经过几个周期，距离理想状态还存在一定的偏差时，需要加大Ki，需要注意的是引入越大的Ki，会导致更大的累积误差，所以可以使用带死区或者变积分，看下面。
    # PID的微分，会让PID的调整曲线，更快得继续下去，也就是说会预测反馈量变化，加入反馈量在向下接近理想状态，那么
    # 这时的bias-lbias肯定是负的，所以越大的Kd会导致影响它向下递减，也就是在稳定PID的上一次量，它的作用是稳定PID曲线
    # 当PID曲线出现过调，或者欠调、震荡时，可以添加更大的Kd来维持PID的稳定输出。

    #什么是过渡时间？过渡时间是从控制开始到反馈量无线逼近理想状态这段时间，他表示了PID控制的快速性和有效性。

    # 输出的控制量为什么要用减的方式进行？ 因为我们假设所有的PID参数 Kp，Ki，Kd都是正的，系数是正的表示正比关系。
    # 所以如果偏差是负，那么得到的控制量也是负的，但是负的偏差意味着我们要加，而不是减，所以必须用减得的方式输出控制量，即下面的tmp -= temp

    pid = PID_loc(0.96,0.31,0.05)
    pid.PID_stable(x)
    # 首先输入到观测值为0，要求得到平稳状态x=200，
    # 同时可发现该例子的控制量输出就是直接得到响应的观测量输入，tmp -= temp
    # 然后得到新的观测反馈，输入到PID，生成新的控制量，这样往复。
    tmp = 0.0
    l1.append(tmp)
    temp = pid.PID_pop(tmp)
    tmp -= temp
    l1.append(tmp)
    for i in range(0,10):
        temp = pid.PID_pop(tmp)
        tmp -= temp
        l1.append(tmp)


# 增量PID，简单PID实现，增量式的表达结果和最近三次的偏差有关，这样就大大提高了系统的稳定性。
class PID_incre(object):
    '''增量PID，无死区，简单PID实现。
    参数说明
    __stable 是PID调节到目标状态（平稳状态），该状态有可能随时间发生变化，可以通过PID_stable来设置，平稳状态
    __lcontrol 是上一次的控制输出量
    __bias 是偏差即实际反馈值与目标值的偏差，反映了差别程度
    __lbias 上一次的偏差历史，在求离散系统中微分有用
    __llbias 上上一次的偏差历史，在求离散系统中微分有用
    __Kp PID的比例系数，非负
    __Ki PID的积分系数，非负
    __Kd PID的微分系数，非负'''
    __stable = 0.0
    __lcontrol = 0.0
    __bias = 0.0
    __lbias = 0.0
    __llbias = 0.0
    __Kp = 0.0
    __Ki = 0.0
    __Kd = 0.0

    def __init__(self,Kp=0.2,Ki=0.2,Kd=0.2):
        self.__Kp = Kp
        self.__Ki = Ki
        self.__Kd = Kd

    def PID_pop(self,actual=100.0):
        self.__bias = actual - self.__stable
        self.__lcontrol += self.__Kp*(self.__bias-self.__lbias) + self.__Ki*self.__bias + self.__Kd*(self.__bias-self.__lbias*2+self.__llbias)
        self.__llbias = self.__lbias
        self.__lbias = self.__bias
        return self.__lcontrol

    def PID_stable(self,stable=100):
        self.__stable = stable

def PID_incre_test(x,l1):
    pid = PID_incre(0.96,0.31,0.05)
    pid.PID_stable(x)
    tmp = 0.0
    l1.append(tmp)
    temp = pid.PID_pop(tmp)
    tmp -= temp
    l1.append(tmp)
    for i in range(0,10):
        temp = pid.PID_pop(tmp)
        tmp -= temp
        l1.append(tmp)


# 增量PID，带死区控制，即震荡大于理想状态的20%不进行正积分，只进行负积分，同时震荡小于理想状态20%只进行正积分。
# 增加变积分，因为在PID的开始一直引入积分会增加震荡的概率，因为往往PID的开始都是和理想状态差距很大的，所以会累加大量的积分因素
# 为了改变它，以时间为原则，Ki的数值与时间陈正比，最后到达我们预设的Ki大小并不改变。
# 以下是程序实现
class PID_increDT(object):
    '''增量PID，无死区，简单PID实现。
    参数说明
    __stable 是PID调节到目标状态（平稳状态），该状态有可能随时间发生变化，可以通过PID_stable来设置，平稳状态
    __lcontrol 是上一次的控制输出量
    __bias 是偏差即实际反馈值与目标值的偏差，反映了差别程度
    __lbias 上一次的偏差历史，在求离散系统中微分有用
    __llbias 上上一次的偏差历史，在求离散系统中微分有用
    __Kp PID的比例系数，非负
    __Ki PID的积分系数，非负
    __Kd PID的微分系数，非负
    __dline 表示死区控制的 上下浮动百分比，0.2即20% 1即100% 2即200%
    __itime 表示积分的变化时效，随时间进行成正比，随后到达预设积分系数大小，不变,这种情况特别适合刚开始偏差很小的情况，对于刚开始偏差比较大的反而会减弱静差的消除
    __tKi 用来存放变积分'''
    __stable = 0.0
    __lcontrol = 0.0
    __bias = 0.0
    __lbias = 0.0
    __llbias = 0.0
    __Kp = 0.0
    __Ki = 0.0
    __Kd = 0.0
    __dline = 0.0
    __itime = 0
    __tKi = 0.0

    def __init__(self,Kp=0.2,Ki=0.2,Kd=0.2,dline=0.2,itime=10):
        self.__Kp = Kp
        self.__Ki = Ki
        self.__Kd = Kd
        self.__dline = dline
        self.__itime = itime

    def PID_pop(self,actual=100.0):
        self.__bias = actual - self.__stable
        #为了忽略精度丢失
        #if self.__Ki-self.__tKi < 0.00001 or self.__tKi-self.__Ki < 0.00001:
            #self.__tKi += self.__Ki/self.__itime
        #if actual < self.__stable-self.__stable*self.__dline or actual > self.__stable+self.__stable*self.__dline:
        if abs(self.__bias) > self.__dline*abs(self.__stable):
            self.__lcontrol += self.__Kp*(self.__bias-self.__lbias) + self.__Kd*(self.__bias-self.__lbias*2+self.__llbias)
        else:
            #为了忽略精度丢失
            if self.__tKi < self.__Ki:
                self.__tKi += self.__Ki/self.__itime
            self.__lcontrol += self.__Kp*(self.__bias-self.__lbias) + self.__tKi*self.__bias + self.__Kd*(self.__bias-self.__lbias*2+self.__llbias)
        self.__llbias = self.__lbias
        self.__lbias = self.__bias
        return self.__lcontrol

    def PID_stable(self,stable=100):
        self.__stable = stable

def PID_increDT_test(x,l1):
    pid = PID_increDT(0.96,0.31,0.05,1,3)
    pid.PID_stable(x)
    tmp = 0.0
    l1.append(tmp)
    temp = pid.PID_pop(tmp)
    tmp -= temp
    l1.append(tmp)
    for i in range(0,10):
        temp = pid.PID_pop(tmp)
        tmp -= temp
        l1.append(tmp)

# 增量PID，带死区控制，即震荡大于理想状态的20%不进行正积分，只进行负积分，同时震荡小于理想状态20%只进行正积分。
# 增加变积分，因为在PID的开始一直引入积分会增加震荡的概率，因为往往PID的开始都是和理想状态差距很大的，所以会累加大量的积分因素
# 为了改变它，通过对比实际值与理想状态的差距百分比，来决定积分量，当然越接近积分越接近预设定值。消除静差越有效。这种情况特别适合刚开始偏差很大的情况，对于刚开始偏差比较小的反而会减弱静差的消除
# 随后加入了时间作为权值，变积分大小与时间成反比，极大了减小了累积偏差。
# 以下是程序实现
class PID_increDTT(object):
    '''增量PID，无死区，简单PID实现。
    参数说明
    __stable 是PID调节到目标状态（平稳状态），该状态有可能随时间发生变化，可以通过PID_stable来设置，平稳状态
    __lcontrol 是上一次的控制输出量
    __bias 是偏差即实际反馈值与目标值的偏差，反映了差别程度
    __lbias 上一次的偏差历史，在求离散系统中微分有用
    __llbias 上上一次的偏差历史，在求离散系统中微分有用
    __Kp PID的比例系数，非负
    __Ki PID的积分系数，非负
    __Kd PID的微分系数，非负
    __dline 表示死区控制的 上下浮动百分比，0.2即20% 1即100% 2即200%
    __itime 表示时间在变积分上的权值，只有时间越长，变积分才会逐渐变大
    __tKi 用来存放变积分,修改了结果是根据偏差值在死区面积占的比例，来决定积分系数的大小'''
    __stable = 0.0
    __lcontrol = 0.0
    __bias = 0.0
    __lbias = 0.0
    __llbias = 0.0
    __Kp = 0.0
    __Ki = 0.0
    __Kd = 0.0
    __dline = 0.0
    __itime = 0.0
    __tKi = 0.0

    def __init__(self,Kp=0.2,Ki=0.2,Kd=0.2,dline=0.2,itime=10):
        self.__Kp = Kp
        self.__Ki = Ki
        self.__Kd = Kd
        self.__dline = dline
        self.__itime = itime

    def PID_pop(self,actual=100.0):
        self.__bias = actual - self.__stable
        #if actual < self.__stable-self.__stable*self.__dline or actual > self.__stable+self.__stable*self.__dline:
        if abs(self.__bias) > abs(self.__stable)*self.__dline:
            self.__lcontrol += self.__Kp*(self.__bias-self.__lbias) + self.__Kd*(self.__bias-self.__lbias*2+self.__llbias)
        else:
            #只有在死区之内，才会求积分
            #为了忽略精度丢失
            self.__tKi = self.__Ki*(1-abs(self.__bias)/abs(self.__stable*self.__dline))*(1.0/self.__itime)
            if self.__itime > 1:
                self.__itime -= 1
            self.__lcontrol += self.__Kp*(self.__bias-self.__lbias) + self.__tKi*self.__bias + self.__Kd*(self.__bias-self.__lbias*2+self.__llbias)
        self.__llbias = self.__lbias
        self.__lbias = self.__bias
        return self.__lcontrol

    def PID_stable(self,stable=100):
        self.__stable = stable

def PID_increDTT_test(x,l1):
    pid = PID_increDTT(0.96,0.31,0.05,1,3)
    pid.PID_stable(x)
    tmp = 0.0
    l1.append(tmp)
    temp = pid.PID_pop(tmp)
    tmp -= temp
    l1.append(tmp)
    for i in range(0,10):
        temp = pid.PID_pop(tmp)
        tmp -= temp
        l1.append(tmp)

if __name__ == '__main__':
    '''
    PID所控制的系统，大部分的系统属于非线性系统， 或者说是系统模型不确定的系统，所以参数整定是很重要的
    同时如果是模型确定的，我们可以直接建模处理哦，不需要这种需要大量调定参数的PID来实现
    '''
    x = 200
    #位置型PID 测试
    #l1 = []
    #PID_loc_test(x,l1)
    #print(l1)
    #xl1 = numpy.array(l1)

    #增量型PID 测试
    l2 = []
    PID_incre_test(x,l2)
    #print(l2)
    xl2 = numpy.array(l2)

    #带死区控制和变积分（时间权值）的PID测试
    l3 = []
    PID_increDT_test(x,l3)
    print(l3)
    xl3 = numpy.array(l3)

    #带死区控制和变积分（偏差权值+时间权值）的PID测试
    l4 = []
    PID_increDTT_test(x,l4)
    print(l4)
    xl4 = numpy.array(l4)

    pylab.figure()
    #对比增量式简单和位置式简单PID两个数据输出，其实都是一样的。
    #pylab.plot(xl1,'r-',label='PID loc')
    pylab.plot(xl2,'b-',label='PID incre')
    #带死区和变积分效果好太多了，震荡变小，同时去除静差也更快，没有了历史积分的累赘
    pylab.plot(xl3,'y-',label='PID increDT')
    #带死区和变积分,改进了变积分规则（偏差越小，变积分越大，为了快速消除静差；加入时间权值，时间越长，变积分越大，在效果上具有巨大改进）
    pylab.plot(xl4,'r-',label='PID increDTT')
    pylab.axhline(x,color='g',label='truth value')
    pylab.legend()
    pylab.xlabel('Iteration')
    pylab.ylabel('Voltage')
    pylab.show()