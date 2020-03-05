#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from math import pi


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber(
            "camera/rgb/image_raw", Image, self.imageCallback
        )
        self.cmd_vel_pub = rospy.Publisher(
            "cmd_vel_mux/input/teleop", Twist, queue_size=1
        )
        self.twist = Twist()
        self.lastDirection = None
        self.h = 0
        self.w = 0
        self.d = 0

    def imageCallback(self, msg):

        # Detect shape
        shape, direction, blobs_list, hsv, image = self.detectShape(msg)

        if shape == "triangle":
            self.followBlob(direction, blobs_list, image)
            self.lastDirection = direction
        elif shape == "star":
            # stop and terminate
            print("Star - stopping")
            self.cmd_vel_pub.publish(Twist())
            rospy.signal_shutdown("Terminate at Star")
        elif shape == None:
            # No shapes were detected, we will follow yellow based on toggle
            self.followYellow(hsv, image)

    def detectShape(self, msg):
        """
        Detects shapes in red and returns image and hsv
        """

        # Convert image to HSV
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.h, self.w, self.d = image.shape
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create red mask
        mask = cv2.inRange(hsv, (160, 100, 100), (179, 255, 255)) + cv2.inRange(
            hsv, (0, 100, 100), (10, 255, 255)
        )

        # Filter shape
        search_top = (3 * self.h / 4) - 40
        search_bot = self.h
        mask[0:search_top, 0 : self.w] = 0
        mask[search_bot : self.h, 0 : self.w] = 0

        # Determine the shape based on the number for edges
        contours = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]
        shape = None
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            peri = cv2.arcLength(c, True)
            vertices = cv2.approxPolyDP(c, 0.04 * peri, True)
            M = cv2.moments(c)
            print("shape area: " + str(M["m00"]))
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                shapeCentroid = (cx, cy)
                blobAbove = self.blobsAboveShape(shapeCentroid, hsv)
                print("vertices of shape:" + str(vertices))
                if len(vertices) == 3 and self.blobBelow(cx, cy, hsv) == True and blobAbove != None:
                    shape = "triangle"
                    vertices_x = []
                    for i in vertices:
                        vertices_x.append(i[0][0])
                    vertices_x.sort()
                    distance_a = abs(vertices_x[0] - vertices_x[1])
                    distance_b = abs(vertices_x[1] - vertices_x[2])
                    if distance_a > distance_b:
                        print("LEFT")
                        direction = "left"
                    else:
                        print("RIGHT")
                        direction = "right"
                    print(direction)
                    return shape, direction, blobAbove, hsv, image
                elif len(vertices) == 5:
                    shape = "star"
                    

        # None of RBG detected, return yellow
        blob = []
        direction = None
        return shape, direction, blob, hsv, image

        # What if there is no shape detected?
        raise Exception("No shapes detected!")

    def blobsAboveShape(self, shape_centroid, hsv):
        mask = cv2.inRange(hsv, (15, 50, 50), (50, 255, 255))
        search_top = (3 * self.h / 4) - 40
        search_bot = self.h - 20
        mask[0:search_top, 0 : self.w] = 0
        mask[search_bot : self.h, 0 : self.w] = 0
        contours = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]
        centroids = []

        if len(contours) > 0:
            # Find blobs
            for c in contours:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    if cy < shape_centroid[1]:
                        centroids.append((cx, cy))
        if len(centroids) > 0:
            # Display blobs
            cv2.imshow("window_blob", mask)
            cv2.waitKey(3)
            return centroids
        else:
            return None

    def blobBelow(self, x, y, hsv):
        mask = cv2.inRange(hsv, (15, 50, 50), (50, 255, 255))
        search_top = 3 * self.h / 4
        search_bot = self.h
        mask[0:search_top, 0 : self.w] = 0
        mask[search_bot : self.h, 0 : self.w] = 0

        contours = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]
        centroids = []

        if len(contours) > 0:
            # Find blobs
            for c in contours:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    if cy > y:
                        centroids.append((cx, cy))
        if len(centroids) > 0:
            return True
        else:
            return False

    def move(self, cx):
        print("Moving...")
        err = cx - (self.w / 2)
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 150
        self.cmd_vel_pub.publish(self.twist)

    def getYellowMask(self, hsv, threshold):
        # Create mask for yellow
        mask = cv2.inRange(hsv, (15, 50, 50), (50, 255, 255))
        search_top = (3 * self.h / 4) + threshold
        search_bot = (3 * self.h / 4) + threshold * 2
        mask[0:search_top, 0 : self.w] = 0
        mask[search_bot : self.h, 0 : self.w] = 0
        return mask

    def followBlob(self, direction, blobs_list, image):
        print("Following the blob")

        # Find center based on toggle

        centroids = blobs_list

        # filter out only the top 2
        centroids_y = [x[1] for x in centroids]
        while len(centroids) > 2:
            centroids.remove(centroids_y.index(max(centroids_y)))

        centroids_x = [x[0] for x in centroids]
        print(centroids)
        if direction == "right":
            followCenter = centroids[centroids_x.index(max(centroids_x))]
            print("RIGHT")
        elif direction == "left":
            followCenter = centroids[centroids_x.index(min(centroids_x))]
            print("LEFT")
        print(followCenter)

        # Move towards the centroid
        cv2.circle(image, followCenter, 10, (0, 0, 255), -1)
        self.move(followCenter[0])
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def followYellow(self, hsv, image):
        # Filter out based on last color
        mask = self.getYellowMask(hsv, 20)
        if self.lastDirection == "right":
            print("RIGHT")
            mask[0 : (self.w / 4) + 20, :] = 0
        elif self.lastDirection == "left":
            print("LEFT")
            mask[(3 * self.w / 4) + 20 :, :] = 0

        # Follow the biggest yellow blob
        contours = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)
            self.move(cx)

        # Display
        cv2.imshow("window", image)
        cv2.waitKey(3)


rospy.init_node("follower")
follower = Follower()
rospy.spin()
