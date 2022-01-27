#!/usr/bin/env python
import os
import rospy
import cv2
import face_recognition
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospkg
from move_aibo import AiboJointMover
import time

class FaceRecogniser(object):

    def __init__(self):
        rospy.loginfo("Start FaceRecogniser Init process...")
        
        self.aibo_mover = AiboJointMover()
        
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # get the file path for aibo_project_solutions
        self.path_to_package = rospack.get_path('aibo_project_solutions')
    
        self.bridge_object = CvBridge()
        rospy.loginfo("Start camera suscriber...")
        self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw",Image,self.camera_callback)
        
        
        
        rospy.loginfo("Finished FaceRecogniser Init process...Ready")
    
    def camera_callback(self,data):
        
        self.recognise(data)

    def recognise(self,data):
        
        # Get a reference to webcam #0 (the default one)
        try:
            # We select bgr8 because its the OpneCV encoding by default
            video_capture = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        
        # Load a sample picture of each person you want to recognise.
        olive_image_file = os.path.join(self.path_to_package,"person_img/olive_best.png")
        olive_bis_image_file = os.path.join(self.path_to_package,"person_img/olive_bis.png")
        timmy_image_file = os.path.join(self.path_to_package,"person_img/timmy_best.png")
        
        # Get the face encodings for each face in each image file
        # Note that we state here two images from Olive. The more the better detection you will get
        # in different angles.
        
        olive_image = face_recognition.load_image_file(olive_image_file)
        olive_face_encoding = face_recognition.face_encodings(olive_image)[0]
        
        olive_bis_image = face_recognition.load_image_file(olive_bis_image_file)
        olive_bis_face_encoding = face_recognition.face_encodings(olive_bis_image)[0]
        
        timmy_image = face_recognition.load_image_file(timmy_image_file)
        timmy_face_encoding = face_recognition.face_encodings(timmy_image)[0]
        
        
        known_faces = [
                        olive_face_encoding,
                        olive_bis_face_encoding,
                        timmy_face_encoding
                        ]
        
        # Initialize some variables
        face_locations = []
        face_encodings = []
        face_names = []
        process_this_frame = True


        # Grab a single frame of video
        #ret, frame = video_capture.read()
        
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(video_capture, (0, 0), fx=0.5, fy=0.5)
        
        # Only process every other frame of video to save time
        if process_this_frame:
            # Find all the faces and face encodings in the current frame of video
            face_locations = face_recognition.face_locations(small_frame)
            face_encodings = face_recognition.face_encodings(small_frame, face_locations)
    
            if not face_encodings:
                rospy.logwarn("No Faces found, please get closer...")
    
            face_names = []
            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                match = face_recognition.compare_faces(known_faces, face_encoding)
                name = "Unknown"
    
    
                if match[0] or match[1]:
                    name = "Olive"
                    self.give_paw()
                elif not match[0] and not match[1] and match[2]:
                    name = "Timmy"
                    self.wiggle_tail()
                elif (match[0] or match[1]) and match[2]:
                    name = "TimmyAndOlive"
                else:
                    name = "NoOne"

                
                rospy.loginfo(name)
                
                face_names.append(name)
    
        process_this_frame = not process_this_frame
    
        
        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 2
            right *= 2
            bottom *= 2
            left *= 2
    
            # Draw a box around the face
            cv2.rectangle(video_capture, (left, top), (right, bottom), (0, 0, 255), 2)
    
            # Draw a label with a name below the face
            cv2.rectangle(video_capture, (left, bottom - 35), (right, bottom), (0, 0, 255))
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(video_capture, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
            
        
        # Display the resulting image
        cv2.imshow("Image window", video_capture)
        cv2.waitKey(1)
        
        
    def wiggle_tail(self):
        """
        Move the Tail for a Second
        """
        rospy.loginfo("Wiggle Tail!")
        
        tailPan_min = -0.7
        tailPan_max = 0.7
        sleep_time = 0.2
        
        for i in range(3):
            self.aibo_mover.pub_tailPan_position.publish(tailPan_min)
            time.sleep(sleep_time)
            self.aibo_mover.pub_tailPan_position.publish(tailPan_max)
            time.sleep(sleep_time)
            
    
    def give_paw(self):
        """
        Give Paw
        """
        rospy.loginfo("Give Paw!")
        
        left_front_leg_min = -1.57
        left_front_leg_max = 0.0
        sleep_time = 2.0

        self.aibo_mover.pub_legLF1_position.publish(left_front_leg_min)
        time.sleep(sleep_time)
        self.aibo_mover.pub_legLF1_position.publish(left_front_leg_max)
        time.sleep(sleep_time)
        
        rospy.loginfo("END Give Paw!")
        



def main():
    rospy.init_node('face_recognising_python_node', anonymous=True)
   
    line_follower_object = FaceRecogniser()

    rospy.spin()
    cv2.destroyAllWindows()

    
if __name__ == '__main__':
    main()