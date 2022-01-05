from moviepy.editor import *


def accelerate_video(video, result, A):
    """
    Args:
        video 输入视频的名称 -> str
        result 输出视频的名称 -> str
        A 视频加速倍数 -> int
    Returns:
        NULL
    """
    v = VideoFileClip(video)
    duration = v.duration
    new_vd = v.fl_time(lambda t: A * t, apply_to=['mask', 'video', 'audio']).set_end(duration / A)
    new_vd.write_videofile(result)
