import numpy as np
import cv2



def triangulate(R,t,pt1,pt2,k):
    # projection matrix
    pr = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
    pr_mat = np.dot(k,pr)
    P = np.hstack((R,t))
    P1 = np.dot(k,P)
    ch1 = np.array(pt1)
    ch2 = np.array(pt2)
    # making matrix 2xn :
    ch1 = pt1.transpose()
    ch2 = pt2.transpose()
    pcd = cv2.triangulatePoints(pr_mat,P1,ch1,ch2)
    #print(cloud.shape)
    pcd = pcd[:4,:]
    return pcd

# relative scale calculation 
def get_relative_scale(old_pcd,new_pcd):
    siz = min(old_pcd.shape[1],new_pcd.shape[1]) 
    o_c = np.zeros((3,siz))
    n_c = np.zeros((3,siz))
    o_c = old_pcd[:,:siz]
    n_c = new_pcd[:,:siz] 
    o_c1 = np.roll(o_c, axis=1,shift = 1)
    n_c1 = np.roll(n_c, axis=1,shift = 1)
    scale = np.linalg.norm((o_c - o_c1), axis = 0)/(np.linalg.norm(n_c - n_c1, axis = 0) + 1e-8 )
    scale = np.median(scale)
    return scale