  bool avoid(geometry_msgs::Twist &msg) {

        float minLeft = this->regionDistance(M_PI/2, 3*M_PI/10);
        float minLeftFront = this->regionDistance(3*M_PI/10, M_PI/10.0);
        float minFront = this->regionDistance(M_PI/10.0, -M_PI/10.0);
        float minRightFront = this->regionDistance(-M_PI/10.0, -3*M_PI/10);
        float minRight = this->regionDistance(-3*M_PI/10, -M_PI/2);
        float direction = 0.0;
        if(minFront < 1.0/2) {
            if(minLeftFront < minRightFront)
                direction = -0.1;
            else
                direction = 0.1;

            this->lock = true;
            ROS_INFO("WALL FRONT");
            ROS_INFO("%f", minFront);
            if(minFront < 0.8/2) {
                if(minFront < 0.6/2) {
                    msg.linear.x = 0.0;
                    msg.angular.z = direction;
                    ROS_INFO("TURN");
                }
                else {
                    msg.linear.x = 0.07;
                    msg.angular.z = direction;
                    ROS_INFO("Move slow while Turn");
                }
            }
            else
            {
                msg.linear.x = 0.1;
                msg.angular.z = direction;
                ROS_INFO("Start Truning");
            }
            msg.angular.z = msg.angular.z*10;
            return false;
        }
        if(minLeftFront < 0.9/2) {
            this->lock = true;
            ROS_INFO("WALL LEFTFRONT");
            if(minLeftFront < 0.7/2) {
                if(minLeftFront < 0.5/2) {
                    msg.linear.x = 0.0;
                    msg.angular.z = -0.1;
                    ROS_INFO("TURN");
                }
                else {
                    msg.linear.x = 0.07;
                    msg.angular.z = -0.1;
                    ROS_INFO("Move slow while Turn");
                }
            }
            else
            {
                msg.linear.x = 0.1;
                msg.angular.z = -0.1;
                ROS_INFO("Start Truning");
            }
            msg.angular.z = msg.angular.z*10;
            return false;
        }
        if(minRightFront < 0.9/2) {
            this->lock = true;
            ROS_INFO("WALL RIGHTFRONT");
            if(minRightFront < 0.7/2) {
                if(minRightFront < 0.5/2) {
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.1;
                    ROS_INFO("TURN");
                }
                else {
                    msg.linear.x = 0.07;
                    msg.angular.z = 0.1;
                    ROS_INFO("Move slow while Turn");
                }
            }
            else
            {
                msg.linear.x = 0.1;
                msg.angular.z = 0.1;
                ROS_INFO("Start Truning");
            }
            msg.angular.z = msg.angular.z*10;
            return false;
        }
        if(minLeft < 0.5/2) {
            this->lock = true;
            ROS_INFO("WALL LEFT");
            msg.linear.x = 0.1;
            msg.angular.z = -0.1;
            ROS_INFO("TURN");
            msg.angular.z = msg.angular.z*10;
            return false;
        }
        if(minRight < 0.5/2) {
            this->lock = true;
            ROS_INFO("WALL RIGHT");
            msg.linear.x = 0.1;
            msg.angular.z = 0.1;
            ROS_INFO("TURN");
            msg.angular.z = msg.angular.z*10;
            return false;
        }
        this->lock = false;
        msg.angular.z = msg.angular.z*10;
        return false;
    }
