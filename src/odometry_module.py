import math
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

from . import utils
#from . import scale
from .motion import Transition, Motion



LOGGER = getLogger(__name__)

class ORBVisualOdometry(object):
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
                 debug:bool = False):
        
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
        
        self.position = np.zeros((3,1))
        self.orientation = np.eye(3).astype(np.float64)


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
        
        if matcher == "flann":
            
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
        self.debug = debug
        self.orientation = np.eye(3).astype(np.float64)
        
        self.running = False

    def run_odometry_loop(self):
        LOGGER.info("STARTING ODOMETRY LOOP")
        asyncio.create_task(self.orb_visual_odometry())
        self.running = True
        
        
    def check_start(self):
        if not self.running:
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
            except:
                LOGGER.warn("Can't find matches, check ORB parameters. Skipping images.")
                continue
            if len(matches)<100:
                LOGGER.warn("Not enough matches to be trustworthy. Skipping images.")
                continue
            
            try:
                
                E, mask_e = self.get_essential_matrix(old_matches, cur_matches)
                R, t = self.recover_pose(E, mask_e, old_matches, cur_matches)
                R = utils.check_norm(R)
                async with self.motion.lock:
                    trans = Transition(R, t, self.memory.current.time - self.memory.last.time)
                    self.motion.append(trans)
                
            except cv2.error as e:
                LOGGER.error(f"Couldn't recover essential matrix or pose from essential matrix, got {e}")
                continue 
        
            
            if self.debug:
                R, t, dt = await self.get_odometry_values()
                
                self.position += self.orientation.dot(t)
                self.orientation = np.dot(self.orientation, R) ##ici il faut peut etre transposer R mais peut etre pas a cause de l'ordre de la multiplication 
                log_pos = np.concatenate((np.array([self.memory.last.count]), self.position.flatten()), axis=0)
                utils.save_numpy_array_to_file_on_new_line(log_pos.flatten().round(3), "./results/position.txt")
                self.draw_matches_and_write_results(matches, R, t, dt)
                
            if not self.debug:
                _, _ , dt = await self.get_odometry_values()
            #Auto-tune sleeping time with respect to the stream and inference speed
            self.sleep = max(self.sleep+ self.time_between_frames_s-dt,0)
    
            
            await asyncio.sleep(self.sleep)

    def get_keypoints(self):
        self.memory.current.get_keypoints(self.orb)
        self.memory.last.get_keypoints(self.orb)

        
    def get_matches(self):
        if self.matcher_type == "BF":
            matches = self.matcher.match(self.memory.last.p_descriptors, self.memory.current.p_descriptors)
            # matches = sorted(matches, key=lambda x: x.distance)  # faire en sorte de pas en avoir beaucoup

            old_matches = np.array([self.memory.last.p[mat.queryIdx].pt for mat in matches])
            cur_matches = np.array([self.memory.current.p[mat.trainIdx].pt for mat in matches])
            return matches, old_matches, cur_matches
        
        if self.matcher_type == "flann":
            matches = self.flann.knnMatch(self.memory.last.p_descriptors,self.memory.current.p_descriptors,k=2)
            if self.debug:
                LOGGER.info(f"number of matches before ration test is {len(matches)}")
            good_matches = []
            ##Do Lowe's ratio test
            for m, n in matches:
                if m.distance < self.lowe_ratio_threshold * n.distance:
                    good_matches.append(m) 
                    
            if self.debug:
                LOGGER.info(f"number of good_matches after ration test is {len(good_matches)}")
                
            old_matches = np.array([self.memory.last.p[mat.queryIdx].pt for mat in good_matches])
            cur_matches = np.array([self.memory.current.p[mat.trainIdx].pt for mat in good_matches])
            return good_matches, old_matches, cur_matches
        
        
        
    def draw_matches_and_write_results(self, matches, R, t, dt):
        final_img = cv2.drawMatches(self.memory.last.frame, self.memory.last.p,
                                    self.memory.current.frame, self.memory.current.p,
                                    matches[:200],
                                    None,
                                    flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
        
        w_x, w_y, w_z = Rotation.from_matrix(R).as_euler(seq = "XYZ", degrees=True)
        v_x, v_y, v_z = t[0][0], t[1][0], t[2][0]
        text_v = f" v_x: {round(v_x,2)}, v_y: {round(v_y,2)}, v_z: {round(v_z,2)}"
        text_w = f" w_x: {round(w_x,2)}, w_y: {round(w_y,2)}, w_z: {round(w_z,2)}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.0
        color = (0,0,0)  # black
        thickness = 4

        position_v = (100, 50)  # (x, y) coordinates
        position_w = (100, 85)
        cv2.putText(final_img, text_v, position_v, font, font_scale, color, thickness)
        cv2.putText(final_img, text_w, position_w, font, font_scale, color, thickness)
        final_img = utils.draw_perspective_axis(final_img)
        image = Image.fromarray(final_img)
        image.save("./results/old" + str(self.memory.last.count) + ".jpg")
        
        
    def get_essential_matrix(self, old_matches, cur_matches):
        E, mask_e, = cv2.findEssentialMat(old_matches, cur_matches, cameraMatrix=self.camera_matrix,
                                         method=cv2.RANSAC, prob=self.ransac_prob,threshold = self.ransac_threshold)
        return E, mask_e

    def recover_pose(self, E, mask_e, old_matches, cur_matches):
        _, R, t, _ = cv2.recoverPose(E, old_matches, cur_matches, self.camera_matrix, mask = mask_e)
        
        R2 = np.transpose(R) #https://answers.opencv.org/question/31421/opencv-3-essentialmatrix-and-recoverpose/
        return R2, -R2.dot(t)
    
    async def get_angular_velocity(self):
        self.check_start()
        # R, t, dt = await self.get_current_transition_values()
        R, t, dt = await self.get_odometry_values()
        phi, theta, psi = Rotation.from_matrix(R).as_euler(seq="ZXZ", degrees=True)
        return utils.euler_to_angular_rate(phi, theta, psi, dt)
        

    async def get_linear_velocity(self):
        self.check_start()
        _, t, _ = await self.get_odometry_values()
        # return t[0][0]/dt, t[1][0]/dt, t[2][0]/dt doesn't mean much more to divide by dt until having a scale
        return t[0][0], t[1][0], t[2][0]
    
                
    async def update_states(self):
        self.memory.append(await self.get_state())
        if len(self.memory)<2:
            await asyncio.sleep(self.time_between_frames_s) #maybe
            self.memory.append(await self.get_state())
        
        
        #check if the last image is too old so it's unlikely to find matching points
        dt =self.memory.current.time - self.memory.last.time 
        if self.memory.current.time - self.memory.last.time > self.time_between_frames_s*5:
            await asyncio.sleep(self.time_between_frames_s) #maybe
            self.memory.append(await self.get_state())


    async def get_state(self):
        self.count +=1
        res = State(count=self.count)
        res.time = time()
        img = await self.cam.get_image()
        if img is None:
            raise ValueError("The image is empty")
        
        ##TODO: Benchmark those conversions
        pil_image = img.convert('L')
        open_cv_image = np.array(pil_image)
        res.frame = open_cv_image.copy()
        return res

    # def get_pcds(self, R, t):
    #     if self.old_pcd is None:
    #         return scale.triangulate(), None
        
    #     new_pcd = scale.triangulate(R, t, self.memory.last.p, self.memory.current.p, self.camera_matrix)
    #     if self.new_pcd is None:
    #         pass
            
    # def get_scale(self):
    #     if self.memory.last.count<2:
    #         return 1
    #     else:
    #         pass
        
        
class State(object):

    def __init__(self, count):
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
        return self[-1]
    
    @property
    def last(self)-> State:
        return self[-2]
    
    
    @property
    def before_last(self)-> State:
        return self[-3]


