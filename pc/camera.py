import cv2
import time
from queue import Queue, LifoQueue
from threading import Thread
import numpy as np
import zmq
import matplotlib
matplotlib.use('TkAgg') #needed for tkinter GUI
import matplotlib.pyplot as plt

from duckietown_slimremote.helpers import timer
from duckietown_slimremote.networking import make_sub_socket, recv_array


class ThreadedSubCamera(Thread):
    def __init__(self, queue, host):
        Thread.__init__(self)
        context = zmq.Context.instance()
        self.queue = queue
        self.sock = make_sub_socket(for_images=True, context_=context, target=host)

    def run(self):
        # block while waiting for image
        # put image into queue

        keep_running = True
        print("listening for camera images")
        timings = []
        start = time.time()
        while keep_running:
            img = recv_array(self.sock)

            timings, start = timer(timings, start)

            if not self.queue.empty():
                self.queue.get()  # discard last img, only ever keep one
            self.queue.put(img)


class SubCameraMaster():
    # controls and communicates with the threaded sub camera
    def __init__(self, host):
        self.queue = LifoQueue(2)
        self.cam_thread = ThreadedSubCamera(self.queue, host)
        # self.cam_thread.daemon = True
        self.cam_thread.start()
        self.last_img = None

    def get_img_blocking(self):
        self.last_img = self.queue.get(block=True)  # wait for image, blocking
        return self.last_img

    def get_img_nonblocking(self):
        if not self.queue.empty():
            self.last_img = self.queue.get(block=False)  # TO TEST: might fail
            return self.last_img
        else:
            return None

    def stop(self):
        # TODO: implement this
        pass


def cam_window_init():
    ### THIS IS SHITTY AND SLOW, USE OPENCV INSTEAD
    plt.ion()
    img = np.random.uniform(0, 255, (256, 512, 3))
    img_container = plt.imshow(img, interpolation='none', animated=True, label="blah")
    img_window = plt.gca()
    return (img_container, img_window)


def cam_window_update(img, img_container, img_window):
    # THIS IS SHITTY AND SLOW, USE OPENCV INSTEAD
    img_container.set_data(img)
    img_window.plot([0])
    plt.pause(0.001)


def cam_windows_init_opencv(res=(160, 120, 3)):
    cv2.imshow('livecam', np.zeros(res))
    cv2.waitKey(1)


def cam_windows_update_opencv(img):
    cv2.imshow('livecam', img[:,:,::-1])
    cv2.waitKey(1)
