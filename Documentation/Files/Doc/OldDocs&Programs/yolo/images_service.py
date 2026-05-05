import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

bridge = CvBridge()

def ros_to_cv(source: Image, encoding: str = "bgr8") -> np.ndarray:
    """
    Converts a ROS Image message to an OpenCV image (numpy array)
    
    Parameters:
    ----------
    source : Image
        The ROS Image message to convert.
    encoding : str, optional
        The desired encoding of the OpenCV image (default is "bgr8").

    Returns:
    -------
    np.ndarray
        The converted OpenCV image.
    """
    return bridge.imgmsg_to_cv2(source, encoding)
    

def cv_to_ros(source: np.ndarray, encoding: str = "bgr8") -> Image:
    """
    Converts an OpenCV image (numpy array) to a ROS Image message.
    
    Parameters:
    ----------
    source : np.ndarray
        The OpenCV image to convert.
    encoding : str, optional
        The desired encoding of the ROS Image message (default is "bgr8").
    
    Returns:
    -------
    Image
        The converted ROS Image message.
    """
    return bridge.cv2_to_imgmsg(source, encoding)

def compressed_to_cv(source: CompressedImage, encoding: int = cv2.IMREAD_COLOR):
    np_arr = np.frombuffer(source.data, np.uint8)
    return cv2.imdecode(np_arr, encoding)


def resize_image(source: np.ndarray, width: int, height: int) -> np.ndarray:
    """
    Resizes an OpenCV image to the specified width and height.

    Parameters:
    ----------
    source : np.ndarray
        The OpenCV image to resize.
    width : int
        The desired width of the resized image.
    height : int
        The desired height of the resized image.

    Returns:
    -------
    np.ndarray
        The resized OpenCV image.
    """
    return cv2.resize(source, (width, height))