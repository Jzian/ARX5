import numpy as np
from matplotlib import pyplot as plt
import time


class PositionPID(object):

    def __init__(self, target, cur_val, dt, max, min, p, i, d) -> None:
        self.dt = dt  # 循环时间间隔
        self._max = max  # 最大输出限制，规避过冲
        self._min = min  # 最小输出限制
        self.k_p = p  # 比例系数
        self.k_i = i  # 积分系数
        self.k_d = d  # 微分系数

        self.target = target  # 目标值
        self.cur_val = cur_val  # 算法当前PID位置值，第一次为设定的初始位置
        self._pre_error = 0  # t-1 时刻误差值
        self._integral = 0  # 误差积分值

    def calculate(self):
        """
        计算t时刻PID输出值cur_val
        """
        error = self.target - self.cur_val  # 计算当前误差
        # 比例项
        p_out = self.k_p * error  
        # 积分项
        self._integral += (error * self.dt)
        i_out = self.k_i * self._integral
        # 微分项
        derivative = (error - self._pre_error) / self.dt
        d_out = self.k_d * derivative

        # t 时刻pid输出
        output = p_out + i_out + d_out

        # 限制输出值
        if output > self._max:
            output = self._max
        elif output < self._min:
            output = self._min
        
        self._pre_error = error
        self.cur_val = output
        return self.cur_val

    def fit_and_plot(self, count = 200):
        """
            使用PID拟合setPoint
        """
        counts = np.arange(count)
        outputs = []

        for i in counts:
            outputs.append(self.calculate())
            print('Count %3d: output: %f' % (i, outputs[-1]))

        print('Done')
        # print(outputs)

        plt.figure()
        plt.axhline(self.target, c='red')
        plt.plot(counts, np.array(outputs), 'b.')
        plt.ylim(min(outputs) - 0.1 * min(outputs), max(outputs) + 0.1 * max(outputs))
        plt.plot(outputs)
        plt.show()

class IncrementalPID(object):

    def __init__(self,dt, p=None, i=None, d=None,loop=False) -> None:
        if None not in [p,i,d]:
            self.k_p = p  # 比例系数
            self.k_i = i  # 积分系数
            self.k_d = d  # 微分系数
        else: self.k_d=self.k_i=self.kp=0  # 允许初始时不设置pid参数值
        self.dt = dt
        self._target = 0
        self._current = 0
        self._pre_error = 0  # t-1 时刻误差值
        self._pre_pre_error = 0  # t-2 时刻误差值
        self.delta_output = 0
        self._error = None
        # 启动计算线程

    @property
    def target(self):
        return self._target

    @property.setter
    def target(self,target):
        self._target = target

    @property
    def current(self):
        return self._current

    @property.setter
    def current(self,current):
        self._current = current

    @property
    def error(self):
        return self._error

    @property.setter
    def error(self,error):
        self._error = error

    def calcalate(self,loop=False):
        while True:
            if self._error is not None:
                error = self.error
            else:
                error = self._target - self._current
            p_change = self.k_p * (error - self._pre_error)
            i_change = self.k_i * error
            d_change = self.k_d * (error - 2 * self._pre_error + self._pre_pre_error)
            self.delta_output = p_change + i_change + d_change  # 本次增量

            self._pre_pre_error = self._pre_error
            self._pre_error = error
            if not loop: break
            time.sleep(self.dt)

    def fit_and_plot(self, count=200):
        counts = np.arange(count)
        outputs=[]
        for i in counts:
            outputs.append(self.calcalate())
            print('Count %3d: output: %f' % (i, outputs[-1]))

        print('Done')

        plt.figure()
        plt.axhline(self._target, c='red')
        plt.plot(counts, np.array(outputs),'b.')
        plt.ylim(min(outputs)-0.1*min(outputs), max(outputs)+0.1*max(outputs))
        plt.plot(outputs)
        plt.show()