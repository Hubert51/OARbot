import tesseract
import os
import re
import numpy as np
import traceback
import time
import tesseract_ros

Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
UR_joint_names=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
UR_link_names=["shoulder_link","upper_arm_link","forearm_link","wrist_1_link","wrist_2_link","wrist_3_link","ee_link"]

def env_setup():
    # TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]


    # with open("urdf/combined.urdf",'r') as f:
    with open("urdf/combined.urdf",'r') as f:
        combined_urdf = f.read()

    with open("urdf/combined.srdf",'r') as f:
        combined_srdf = f.read()

    t = tesseract.Tesseract()

    t.init(combined_urdf, TesseractSupportResourceLocator())

    t_env = t.getEnvironment()
    print t_env


    contact_distance=0.102

    monitored_link_names = t_env.getLinkNames()
    manager = t_env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(monitored_link_names)
    manager.setContactDistanceThreshold(contact_distance)
    return manager, t_env

def test_environment(manager, t_env, sawyer_joints,ur5_joints,robot_name):
    global Sawyer_joint_names, UR_joint_names, Sawyer_link_names, UR_link_names

    t_env.setState(Sawyer_joint_names, sawyer_joints)
    t_env.setState(UR_joint_names, ur5_joints)


    env_state = t_env.getCurrentState()
    manager.setCollisionObjectsTransform(env_state.transforms)
    contacts = manager.contactTest(2)
    contact_vector = tesseract.flattenResults(contacts)

    distances = np.array([c.distance for c in contact_vector])
    nearest_points=np.array([c.nearest_points for c in contact_vector])
    names = np.array([c.link_names for c in contact_vector])
    # nearest_index=np.argmin(distances)
    # print(names)

    min_UR=9
    min_UR_index=0
    min_Sawyer=9
    min_Sawyer_index=0
    Closest_Pt=[None,None,None]
    Closest_Pt_env=[None,None,None]
    if(robot_name=="UR5"):
        for i in range(len(distances)):
            if (names[i][0] in UR_link_names or names[i][1] in UR_link_names) and distances[i]<min_UR and not (names[i][0] in UR_link_names and names[i][1] in UR_link_names):
                min_UR=distances[i]
                min_UR_index=i

        UR_link_name=0
        if (len(distances)>0):
            if names[min_UR_index][0] in UR_link_names and names[min_UR_index][1] in UR_link_names:
                stop=1
            elif names[min_UR_index][0] in UR_link_names:
                UR_link_name=names[min_UR_index][0]
                Closest_Pt=nearest_points[min_UR_index][0]
                Closest_Pt_env=nearest_points[min_UR_index][1]
                print(names[min_UR_index])
                print(distances[min_UR_index])
            elif names[min_UR_index][1] in UR_link_names:
                UR_link_name=names[min_UR_index][1]
                Closest_Pt=nearest_points[min_UR_index][1]
                Closest_Pt_env=nearest_points[min_UR_index][0]
                print(names[min_UR_index])
                print(distances[min_UR_index])
            
            
            return Closest_Pt,Closest_Pt_env,distances[min_UR_index],UR_link_name
    if(robot_name=="Sawyer"):
        for i in range(len(distances)):
            if (names[i][0] in Sawyer_link_names or names[i][1] in Sawyer_link_names) and distances[i]<min_Sawyer and not (names[i][0] in Sawyer_link_names and names[i][1] in Sawyer_link_names):
                min_Sawyer=distances[i]
                min_Sawyer_index=i
        Sawyer_link_name=0
        if (len(distances)>0):
            if names[min_Sawyer_index][0] in Sawyer_link_names and names[min_Sawyer_index][1] in Sawyer_link_names:
                stop=1
            elif names[min_Sawyer_index][0] in Sawyer_link_names:
                Sawyer_link_name=names[min_Sawyer_index][0]
                Closest_Pt=nearest_points[min_Sawyer_index][0]
                Closest_Pt_env=nearest_points[min_Sawyer_index][1]
                print(names[min_Sawyer_index])
                print(distances[min_Sawyer_index])
            elif names[min_Sawyer_index][1] in Sawyer_link_names:
                Sawyer_link_name=names[min_Sawyer_index][1]
                Closest_Pt=nearest_points[min_Sawyer_index][1]
                Closest_Pt_env=nearest_points[min_Sawyer_index][0]
                print(names[min_Sawyer_index])
                print(distances[min_Sawyer_index])
                
            
            return Closest_Pt,Closest_Pt_env,distances[min_Sawyer_index],Sawyer_link_name
    
    return Closest_Pt,Closest_Pt_env,None,None 

    #TODO: test more stuff...

class TesseractSupportResourceLocator(tesseract.ResourceLocator):
    def __init__(self):
        super(TesseractSupportResourceLocator,self).__init__()
        # self.TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]
        # self.TESSERACT_SUPPORT_DIR = "/home/ruijie/kinova_ws/devel_isolated/tesseract_support/share/tesseract_support"
        self.TESSERACT_SUPPORT_DIR = "/home/ruijie/Desktop/Motion_Planner-master/tesseract_support"
        # self.TESSERACT_SUPPORT_DIR = "/home/ruijie/kinova_ws/devel_isolated/tesseract_support"

    def locateResource(self, url):
        try:
            url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
            if (url_match is None):
                return None
            
            fname = os.path.join(self.TESSERACT_SUPPORT_DIR, os.path.normpath(url_match.group(1)))
            with open(fname,'rb') as f:
                resource_bytes = f.read()

            resource = tesseract.BytesResource(url, resource_bytes)

            return resource
        except:
            traceback.print_exc()
            return

if __name__ == '__main__':

    #UR5 test
    ur_joints=np.array([1.1,-1.68,1.65,-1.54,-1.57,3.59])
    sawyer_joints=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])


    # #######Sawyer test
    sawyer_joints=np.array([-0.8,-0.99,-0.35,2.12,-0.29,0.18,3.52])
    ur_joints=np.array([90,-21.8, 0,-90,-90,204.75])*np.pi/180.
    manager,t_env=env_setup()
    Closest_Pt,Closest_Pt_env, dist, link_name=test_environment(manager, t_env, sawyer_joints,ur_joints,"Sawyer")

    print(Closest_Pt,Closest_Pt_env)

