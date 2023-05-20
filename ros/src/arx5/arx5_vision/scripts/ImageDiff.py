import cv2
import os

def makedir(dir_path):
    dir_path=os.path.dirname(dir_path)#获取路径名，删掉文件名
    bool=os.path.exists(dir_path) #存在返回True，不存在返回False
    if bool:
        pass
    else:
        os.makedirs(dir_path)

# 该方法中f单位为mm
def pix2mm_1(d,imagXsize,type="D455",f=None,sensor_xsize=None,num=1):
    if type=="D455":
        f = 1.93  # mm
        sensor_xsize=2.7288
    elif type=="D435" or type=="D435i":
        f = 1.395 # mm
        sensor_xsize=2.7288
    H = d/f*sensor_xsize
    p2mm = H/imagXsize
    return p2mm

# 该方法中fx为像素焦距,d为物体距离相机的距离（mm）
def pix2mm_2(d,num=1,type="D455",fx=None):
    if type=="D455":
        fx = 642.9619  # mm
    elif type=="D435" or type=="D435i":
        fx = 1395  # mm
    p2mm = d*num/fx
    return p2mm

img_path = "../Images/"
imag1_name = "0_Image_static1.jpg"   # 要包含拓展名
imag2_name = "1_Image_static1.jpg"

flag = 0

if __name__ == "__main__":
    if flag==0:
        makedir(img_path)   # 如果本来没有，则新创建一个
        img1 = cv2.imread(img_path + imag1_name,0)
        img2 = cv2.imread(img_path + imag2_name,0)
        difimg = cv2.absdiff(img1,img2)
        ret,difimg=cv2.threshold(src=difimg,thresh=10,maxval=255,type=cv2.THRESH_BINARY)
        cv2.imwrite(img_path + "NewDiffImag_st1.jpg" , difimg)  # 保存贞差图
        # cv2.namedWindow("Diff",cv2.WINDOW_NORMAL)
        # cv2.imshow("Diff",difimg)
        # cv2.waitKey(0)
        print("贞差图保存完毕\r\n")
    elif flag==1:
        print(pix2mm_2(200,type="D455"))