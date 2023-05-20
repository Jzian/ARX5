from arx5_vision.vision import StdVideo

USE_SIM = True

if USE_SIM:
    StdVideo.color_thresh_determine("camera/color/image_raw")
else:
    StdVideo.color_thresh_determine(2)  # 阻塞，按esc退出