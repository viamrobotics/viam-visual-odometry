import time
from collections import deque
import numpy as np
import cv2
from PIL import Image
from viam.logging import getLogger
from scipy.spatial.transform import Rotation
from viam.components.camera import Camera
from time import time
import asyncio
from viam.media.video import CameraMimeType

from . import utils
#from . import scale
from .motion import Transition, Motion

LOGGER = getLogger(__name__)

class ORBVisualOdometry(object):
    """
    This class has two jobs. 
     * One coroutine requests images from the camera
    and compute relative transitions between two images. 
     * One coroutine access the transition computed by the first coroutine and
     convert it to either a linear velocity or angular velocity. 
    """    
    def __init__(self,
                 cam: Camera,
                 camera_matrix,
                 distortion_param,
                 time_between_frames,
                 n_features:int,
                 edge_threshold:int,
                 patch_size:int,
                 n_levels:int,
                 first_level:int,
                 fast_threshold:int,
                 scale_factor:float,
                 WTA_K:int,
                 matcher:str,
                 lowe_ratio_threshold:int, 
                 ransac_prob:float, 
                 ransac_threshold:float,
                 window:int=5,
                 save_trajectory:bool = False, 
                 log_error_proportion=False):
        self.cam = cam
        self.camera_matrix = camera_matrix
        self.distort_param = distortion_param
        
        self.memory = Memory(maxlen=window)
        self.motion = Motion(maxlen=window)
        # self.trajectory = Trajectory(maxlen=window)

        
        self.time_between_frames_s = time_between_frames
        self.sleep = 0
        self.dt = 0
        
        self.count = -1
        self.count_failed_matches = 0
        self.count_cv2_error = 0
        self.count_old_image_issue = 0
        self.norm_big = 0
        
        self.position = np.zeros((3,1))
        self.orientation_lock = asyncio.Lock()


        self.orb = cv2.ORB_create(nfeatures=n_features,
                                  edgeThreshold=edge_threshold,
                                  patchSize=patch_size,
                                  nlevels=n_levels,
                                  fastThreshold=fast_threshold,
                                  scaleFactor=scale_factor,
                                  WTA_K=WTA_K,
                                  scoreType=cv2.ORB_HARRIS_SCORE,
                                  firstLevel=first_level)
        
        self.matcher_type = matcher 
        
        if matcher == "BF":
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        else:
            
            FLANN_INDEX_LSH = 6
            
            index_params= dict(algorithm = FLANN_INDEX_LSH,
                    table_number = 6,
                    key_size = 12,
                    multi_probe_level = 1)
            search_params = dict(checks=50)
            self.flann = cv2.FlannBasedMatcher(index_params,search_params) 
        
            

        self.lowe_ratio_threshold = lowe_ratio_threshold
        self.ransac_prob = ransac_prob
        self.ransac_threshold = ransac_threshold
        
        self.save_trajectory = save_trajectory
        self.log_error_proportion = log_error_proportion
        self.orientation = np.eye(3).astype(np.float64)
        self.running = False
        self.running_lock = asyncio.Lock()


    def run_odometry_loop(self):
        LOGGER.info("STARTING ODOMETRY LOOP")
        self.task = asyncio.create_task(self.orb_visual_odometry())
        
        
    async def check_start(self):
        async with self.running_lock:
            if not self.running:
                self.running = True
                self.run_odometry_loop()

    async def get_current_transition_values(self):
        return await self.motion.get_current_transition_values()
    
    async def get_odometry_values(self):
        async with self.motion.lock:
            R, t, dt = self.motion.current.R, self.motion.current.t, self.motion.current.dt
        return R, t, dt 

    async def orb_visual_odometry(self):
        while True:
            await self.update_states()
            self.get_keypoints()

            try:
                matches, old_matches, cur_matches = self.get_matches()
            except cv2.error as e:
                LOGGER.debug(f"couldn't get matches got {e}")
                continue
            
            if len(matches)<30:
                if self.log_error_proportion:
                    self.count_failed_matches +=1
                LOGGER.debug("Not enough matches to be trustworthy. Skipping images.")
                async with self.motion.lock:
                        R = self.motion.current.R
                #update orientation with last rotation:         
                async with self.orientation_lock:
                    self.orientation = np.dot(self.orientation, R)  
                continue
                
                
            else:
                try:
                    
                    E, mask_e = self.get_essential_matrix(old_matches, cur_matches)
                    R, t = self.recover_pose(E, mask_e, old_matches, cur_matches)
                    
                                    
                except cv2.error as e:
                    if self.log_error_proportion:
                        self.count_cv2_error +=1
                    LOGGER.debug(f"Couldn't recover essential matrix or pose from essential matrix, got {e}")
                
                    async with self.motion.lock:
                        R = self.motion.current.R
                
                    #update orientation with last rotation:         
                    async with self.orientation_lock:
                        self.orientation = np.dot(self.orientation, R)
                    continue
                
                R, norm, _, _, _ = utils.check_norm(R, 100)
                
                if norm>100:
                    self.norm_big +=1
                    async with self.motion.lock:
                        R = self.motion.current.R
                    # update orientation with last rotation:  
                    async with self.orientation_lock:
                        self.orientation = np.dot(self.orientation, R)  
                    continue

                #update orientation with new rotation:        
                async with self.orientation_lock:
                    self.orientation = np.dot(self.orientation, R)

                #update odometry values
                async with self.motion.lock:
                    trans = Transition(R, t, self.memory.current.time - self.memory.last.time)
                    self.motion.append(trans)

            if self.save_trajectory:
                R, t, _ = await self.get_odometry_values()
                self.position += self.orientation.dot(t)
                log_pos = np.concatenate((np.array([self.memory.last.count]), self.position.flatten()), axis=0)
                utils.save_numpy_array_to_file_on_new_line(log_pos.flatten().round(3), "./results/position.txt")
                utils.draw_matches_and_write_results(self.memory.last, self.memory.current, matches,R, t)
                
# 
            _, _ , dt = await self.get_odometry_values()
                
            # Auto-tune sleeping time with respect to the stream and inference speed
            LOGGER.error(f'ITERATION {self.count}')
            LOGGER.error(f"last sleep is : {self.sleep}")
            LOGGER.error(f"time between images  is : {dt}")
            # self.sleep = self.sleep + self.time_between_frames_s-dt
            self.sleep += self.time_between_frames_s-self.sleep
            LOGGER.error(f"sleep is : {self.sleep}")
            
            
            await asyncio.sleep(max(self.sleep,0))
            if self.log_error_proportion:
                LOGGER.debug(f'ITERATION {self.count}')
                LOGGER.debug(f"Failed matches {self.count_failed_matches/self.count * 100}%")
                LOGGER.debug(f"Failed cv2 {self.count_cv2_error/self.count * 100}%")
                LOGGER.debug(f"Failed old_image {self.count_old_image_issue/self.count * 100}%")
                LOGGER.debug(f"Failed norm {self.norm_big/self.count * 100}%")

    def get_keypoints(self):
        self.memory.current.get_keypoints(self.orb)
        self.memory.last.get_keypoints(self.orb)
        
    def get_matches(self):
        if self.matcher_type == "BF":
            matches = self.matcher.match(self.memory.last.p_descriptors, self.memory.current.p_descriptors)
            old_matches = np.array([self.memory.last.p[mat.queryIdx].pt for mat in matches])
            cur_matches = np.array([self.memory.current.p[mat.trainIdx].pt for mat in matches])
            return matches, old_matches, cur_matches
        
        if self.matcher_type == "flann":
            matches = self.flann.knnMatch(self.memory.last.p_descriptors,self.memory.current.p_descriptors,k=2)
            # if self.log_error_proportion:
            #     LOGGER.info(f"number of matches before ratio test is {len(matches)}")
            good_matches = []
            
            #perform Lowe's ration test
            for match in matches:
                if len(match) == 2:
                    m, n = match
                    if m.distance < self.lowe_ratio_threshold * n.distance:
                        good_matches.append(m)
                        
                #sometimes knn finds only one good match
                #in this case, we keep it 
                elif len(match) ==1 :
                    # continues
                    good_matches.append(match[0])
                                
                else:
                    continue
                
            # if self.log_error_proportion:
            #     LOGGER.info(f"number of good_matches after ratio test is {len(good_matches)}")
                
            old_matches = np.array([self.memory.last.p[mat.queryIdx].pt for mat in good_matches])
            cur_matches = np.array([self.memory.current.p[mat.trainIdx].pt for mat in good_matches])
            return good_matches, old_matches, cur_matches
        
        
    def get_essential_matrix(self, old_matches, cur_matches):
        E, mask_e, = cv2.findEssentialMat(old_matches, cur_matches, cameraMatrix=self.camera_matrix,
                                         method=cv2.RANSAC, prob=self.ransac_prob,threshold = self.ransac_threshold)
        return E, mask_e

    def recover_pose(self, E, mask_e, old_matches, cur_matches):
        _, R, t, _ = cv2.recoverPose(E, old_matches, cur_matches, self.camera_matrix, mask = mask_e)
        
        R2 = np.transpose(R) #https://answers.opencv.org/question/31421/opencv-3-essentialmatrix-and-recoverpose/
        return R2, -R2.dot(t)
    
    async def get_angular_velocity(self):
        await self.check_start()
        R, t, dt = await self.get_odometry_values()
        phi, theta, psi = Rotation.from_matrix(R).as_euler(seq="ZXZ", degrees=True)
        return utils.euler_to_angular_rate(phi, theta, psi, dt)
        

    async def get_linear_velocity(self):
        await self.check_start()
        _, t, _ = await self.get_odometry_values()
        # return t[0][0]/dt, t[1][0]/dt, t[2][0]/dt doesn't mean much more to divide by dt until having a scale
        return t[0][0], t[1][0], t[2][0]
    
    async def get_orientation(self):
        await self.check_start()
        async with self.orientation_lock:
            orientation = self.orientation
        return orientation
                    
    async def update_states(self):
        self.memory.append(await self.get_state())
        if len(self.memory)<2:
            await asyncio.sleep(self.time_between_frames_s) #maybe
            self.memory.append(await self.get_state())
        
        
        #check if the last image is too old so it's unlikely to find matching points
        dt =self.memory.current.time - self.memory.last.time 
        if dt > self.time_between_frames_s*10:
            if self.log_error_proportion:
                self.count_old_image_issue +=1
            LOGGER.debug("too old image, getting a new one")
            self.memory.append(await self.get_state())


    async def get_state(self):
        self.count +=1
        res = State(count=self.count)
        res.time = time()
        img = await self.cam.get_image(mime_type=CameraMimeType.JPEG)
        if img is None:
            raise ValueError("The image is empty")
        
        ##TODO: Benchmark those conversions
        pil_image = img.convert('L')
        open_cv_image = np.array(pil_image)
        res.frame = open_cv_image.copy()
        return res
    
        
class State(object):

    def __init__(self, count=0):
        self.count = count
        self.time = None
        self.frame = None
        self.p = None
        self.p_descriptors = None
        self.pcd = None

    def get_keypoints(self, orb):
        if self.p is None:
            self.p, self.p_descriptors = orb.detectAndCompute(self.frame, None)
        


class Memory(deque):
    '''
    Class to store past states
    '''
    
    def __init__(self,
                 maxlen: int):
        super().__init__(maxlen=maxlen)
    
    @property    
    def current(self) -> State:
        if len(self)>0:
            return self[-1]
        else:
            return State()
    
    @property
    def last(self)-> State:
        return self[-2]
    
    
    @property
    def before_last(self)-> State:
        return self[-3]


