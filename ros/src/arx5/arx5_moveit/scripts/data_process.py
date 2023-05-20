from scipy import signal
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
import numpy as np

class Filter(object):
    """ 进行滤波用的类 """
    def filt():
        """
        参数说明：
        1.函数 scipy.signal.filtfilt(b, a, x, axis=-1, padtype='odd', padlen=None, method='pad', irlen=None)
            输入参数：
                b: 滤波器的分子系数向量
                a: 滤波器的分母系数向量
                x: 要过滤的数据数组。（array型）
                axis: 指定要过滤的数据数组x的轴
                padtype: 必须是“奇数”、“偶数”、“常数”或“无”。这决定了用于过滤器应用的填充信号的扩展类型。{‘odd', ‘even', ‘constant', None}
                padlen：在应用滤波器之前在轴两端延伸X的元素数目。此值必须小于要滤波元素个数- 1。（int型或None）
                method：确定处理信号边缘的方法。当method为“pad”时，填充信号；填充类型padtype和padlen决定，irlen被忽略。当method为“gust”时，使用古斯塔夫森方法，而忽略padtype和padlen。{“pad” ，“gust”}
                irlen：当method为“gust”时，irlen指定滤波器的脉冲响应的长度。如果irlen是None，则脉冲响应的任何部分都被忽略。对于长信号，指定irlen可以显著改善滤波器的性能。（int型或None）
            输出参数：
                y:滤波后的数据数组
        2.函数 scipy.signal.butter(N, Wn, btype='low', analog=False, output='ba')
            输入参数：
                N:滤波器的阶数
                Wn：归一化截止频率。计算公式Wn=2*截止频率/采样频率。（注意：根据采样定理，采样频率要大于两倍的信号本身最大的频率，才能还原信号。截止频率一定小于信号本身最大的频率，所以Wn一定在0和1之间）。当构造带通滤波器或者带阻滤波器时，Wn为长度为2的列表。
                btype : 滤波器类型{‘lowpass', ‘highpass', ‘bandpass', ‘bandstop'},
                output : 输出类型{‘ba', ‘zpk', ‘sos'},
            输出参数：
                b，a: IIR滤波器的分子（b）和分母（a）多项式系数向量。output='ba'
                z,p,k: IIR滤波器传递函数的零点、极点和系统增益. output= 'zpk'
                sos: IIR滤波器的二阶截面表示。output= 'sos'
        """

        #####1.准备数据
        y = [19.4,18.5,19.2,18.7,18.5,19.1,19.1,19.2,19.3,19.3,19.1,18.7,19.4,18.9,18.7,19.4,19.0,19.6,19.7,19.5,19.8,19.8,19.3,19.5,19.7,19.7,
            19.5,19.4,19.5,20.1,19.3,19.7,20.2,19.5,19.3,19.6,19.1,19.8,18.8,19.1,19.3,18.7,19.4,19.1,18.7,19.2,19.0,18.4,18.6,18.5,19.4,
            19.1,18.8,18.4,18.8,19.4,18.8,18.3,19.1,18.4,19.1,19.4,18.5,19.2,18.7,18.5,19.1,19.1,19.2,19.3,19.3,19.1,18.7,19.4,18.9,18.7,
            19.4,19.0,19.6,19.7,19.5,19.8,19.8,19.3,19.5,19.7,19.7,19.5,19.4,19.5,20.1,19.3,19.7,20.2,19.5,19.3,19.6,19.1,19.8,18.8,19.1,
            19.3,18.7,19.4,19.1,18.7,19.2,19.0,18.4,18.6,18.5,19.4,19.1,18.8,18.4,18.8,19.4,18.8,18.3,19.1,18.4,19.1]
        
        x = [i for i in range(len(y))]

        #####2.低通滤波
        b, a = signal.butter(8, 0.1, 'lowpass')  #8表示滤波器的阶数
        filtedData = signal.filtfilt(b, a, y)
        plt.plot(x, y)
        plt.plot(x, filtedData)
        plt.show()
        
        #####3.高通滤波
        b, a = signal.butter(8, 0.2, 'highpass')  #8表示滤波器的阶数
        filtedData = signal.filtfilt(b, a, y)
        plt.plot(x, y)
        plt.plot(x, filtedData)
        plt.show()
        
        #####4.带通滤波
        b, a = signal.butter(8, [0.4, 0.6], 'bandpass')  #8表示滤波器的阶数
        filtedData = signal.filtfilt(b, a, y)
        plt.plot(x, y)
        plt.plot(x, filtedData)
        plt.show()
        
        #####5.带阻滤波
        b, a = signal.butter(8, [0.4, 0.6], 'bandstop')  #8表示滤波器的阶数
        filtedData = signal.filtfilt(b, a, y)
        plt.plot(x, y)
        plt.plot(x, filtedData)
        plt.show()

class Interpolate(object):
    """ 进行轨迹插值用的类 """
    @staticmethod
    def __time_clip(start,end,interval,unit='s',end_control=False):
        """ ms级的时间细化 """
        if unit == 's': precision = 0.001
        elif unit == 'ms': precision = 1
        time_line = (np.array([start,end,interval])/precision).astype('int32')
        discretized_range = np.arange(time_line[0],time_line[1],step=time_line[2])
        if end_control:
            if discretized_range[-1] != time_line[1]:
                discretized_range = np.append(discretized_range,time_line[1])
        else: discretized_range = np.append(discretized_range,time_line[1])
        discretized_range = discretized_range.astype('float64')
        discretized_range*=precision
        return discretized_range
    @classmethod
    def spline(cls,t,y,t_inc,k=5,unit='s',sort=False,plot=False):
        """
        使用 UnivariateSpline 实现 1-5次样条插值，常用3次和5次
        参数:
            t: 时间序列，一个一维数组
            y: 时间序列对应的函数值，一个一维数组
        返回值:
            返回一个二元组，包含插值后的时间序列和对应的函数值
        """
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t = t[idx]
            y = y[idx]
        # 对插值后的时间序列进行扩展
        t_interp = cls.__time_clip(t[0],t[-1],t_inc,unit)
        # 使用 UnivariateSpline 进行 5次样条插值
        y_interp = UnivariateSpline(t,y,k=k)(t_interp)
        if plot: cls.plot(t,y,t_interp,y_interp)
        # 返回插值后的时间序列和函数值
        return t_interp, y_interp
    @staticmethod
    def plot(t,y,t_interp,y_interp):
        """
        绘制样条插值的结果
        参数:
            t: 时间序列，一个一维数组
            y: 时间序列对应的函数值，一个一维数组
            t_interp: 插值后的时间序列，一个一维数组
            y_interp: 插值后的时间序列对应的函数值，一个一维数组
        """
        # 绘制原始数据点
        plt.plot(t, y,'ro',label='original',markersize=5)
        # 绘制样条插值的结果
        plt.plot(t_interp,y_interp,'g-',label='spline',markersize=3)
        # 添加图例和标签
        plt.legend(loc='best')
        plt.xlabel('t')
        plt.ylabel('y')
        plt.title('Spline Interpolation')
        # 显示图形
        plt.show()

if __name__ == '__main__':
    # 构造测试数据
    t = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
    y = np.array([0, 1, 4, 9, 16, 25, 36, 49, 64, 81])
    # 进行样条插值
    t_interp, y_interp = Interpolate.spline(t,y,0.5,k=5,plot=True)
